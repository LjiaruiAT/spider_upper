#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {

bool is_zero_command(const geometry_msgs::msg::Twist& cmd) {
    constexpr double kEpsilon = 1e-6;
    return std::fabs(cmd.linear.x) < kEpsilon &&
           std::fabs(cmd.linear.y) < kEpsilon &&
           std::fabs(cmd.angular.z) < kEpsilon;
}

geometry_msgs::msg::Twist make_zero_twist() {
    return geometry_msgs::msg::Twist();
}

}  // namespace

class SpiderTaskContext {
public:
    struct CommandSnapshot {
        geometry_msgs::msg::Twist command{};
        uint64_t version{0};
        bool fresh{false};
    };

    SpiderTaskContext(
        rclcpp::Node* node,
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher,
        double timeout_sec)
        : node_(node), publisher_(std::move(publisher)), timeout_sec_(timeout_sec) {}

    rclcpp::Node& node() const { return *node_; }

    void update_latest_cmd(const geometry_msgs::msg::Twist& msg, const rclcpp::Time& stamp) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_cmd_ = msg;
        last_cmd_stamp_ = stamp;
        ++command_version_;
        has_latest_cmd_ = true;
    }

    CommandSnapshot snapshot_latest_command(const rclcpp::Time& now) const {
        std::lock_guard<std::mutex> lock(mutex_);

        CommandSnapshot snapshot;
        snapshot.command = latest_cmd_;
        snapshot.version = command_version_;
        snapshot.fresh = false;

        if (!has_latest_cmd_) {
            return snapshot;
        }

        const auto age = now - last_cmd_stamp_;
        const double age_sec = static_cast<double>(age.nanoseconds()) * 1e-9;
        snapshot.fresh = (age_sec >= 0.0) && (age_sec <= timeout_sec_);
        return snapshot;
    }

    void publish_task_cmd_vel(const geometry_msgs::msg::Twist& msg) const {
        if (publisher_) {
            publisher_->publish(msg);
        }
    }

    void publish_zero_task_cmd_vel() const {
        if (publisher_) {
            publisher_->publish(make_zero_twist());
        }
    }

    uint64_t forwarded_version() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return forwarded_version_;
    }

    void mark_forwarded(uint64_t version) {
        std::lock_guard<std::mutex> lock(mutex_);
        forwarded_version_ = version;
    }

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    mutable std::mutex mutex_;
    geometry_msgs::msg::Twist latest_cmd_{};
    rclcpp::Time last_cmd_stamp_{};
    uint64_t command_version_{0};
    uint64_t forwarded_version_{0};
    bool has_latest_cmd_{false};
    double timeout_sec_{0.25};
};

class BaseTask {
public:
    BaseTask(SpiderTaskContext* context, std::string name)
        : context_(context), task_name_(std::move(name)) {}
    virtual ~BaseTask() = default;

    virtual void on_enter(const std::string& last_task_name) { (void)last_task_name; }
    virtual std::string process(const std::string& last_task_name) = 0;

    SpiderTaskContext* context_;
    std::string task_name_;
};

class StandTask : public BaseTask {
public:
    StandTask(SpiderTaskContext* context, std::string name)
        : BaseTask(context, std::move(name)) {}

    void on_enter(const std::string& last_task_name) override {
        (void)last_task_name;
        context_->publish_zero_task_cmd_vel();
        RCLCPP_INFO(
            context_->node().get_logger(),
            "Entered task [%s], publishing zero /spider/task_cmd_vel",
            task_name_.c_str());
    }

    std::string process(const std::string& last_task_name) override {
        (void)last_task_name;
        const auto snapshot = context_->snapshot_latest_command(context_->node().now());
        if (snapshot.fresh && !is_zero_command(snapshot.command)) {
            return "move_request";
        }

        RCLCPP_INFO_THROTTLE(
            context_->node().get_logger(),
            *context_->node().get_clock(),
            1000,
            "Task [%s] holding stand state",
            task_name_.c_str());
        return "stand";
    }
};

class MoveRequestTask : public BaseTask {
public:
    MoveRequestTask(SpiderTaskContext* context, std::string name)
        : BaseTask(context, std::move(name)) {}

    void on_enter(const std::string& last_task_name) override {
        (void)last_task_name;
        const auto snapshot = context_->snapshot_latest_command(context_->node().now());
        if (snapshot.fresh && !is_zero_command(snapshot.command)) {
            context_->publish_task_cmd_vel(snapshot.command);
            context_->mark_forwarded(snapshot.version);
            RCLCPP_INFO(
                context_->node().get_logger(),
                "Entered task [%s], forwarding latest /spider/task_cmd_vel",
                task_name_.c_str());
        }
    }

    std::string process(const std::string& last_task_name) override {
        (void)last_task_name;

        const auto snapshot = context_->snapshot_latest_command(context_->node().now());
        if (!snapshot.fresh || is_zero_command(snapshot.command)) {
            return "stand";
        }

        if (snapshot.version != context_->forwarded_version()) {
            context_->publish_task_cmd_vel(snapshot.command);
            context_->mark_forwarded(snapshot.version);

            RCLCPP_INFO_THROTTLE(
                context_->node().get_logger(),
                *context_->node().get_clock(),
                1000,
                "Task [%s] forwarding /spider/task_cmd_vel: vx=%.3f, vy=%.3f, wz=%.3f",
                task_name_.c_str(),
                snapshot.command.linear.x,
                snapshot.command.linear.y,
                snapshot.command.angular.z);
        }

        return "move_request";
    }
};

// 这是一个最小的 ROS2 任务调度节点：
// 当前负责接收高层速度命令 /spider/cmd_vel，
// 再通过显式任务状态机转发成 /spider/task_cmd_vel。
// 后续步态、轨迹和 IK 逻辑会继续下沉到 leg_calc。
class SpiderTaskNode : public rclcpp::Node {
public:
    SpiderTaskNode()
        : Node("spider_task_node"), task_manager_initialized_(false), current_task_name_("stand"), last_task_name_("stand"), shutdown_requested_(false) {
        declare_parameter<double>("cmd_vel_timeout_sec", 0.25);
        declare_parameter<double>("task_loop_hz", 50.0);

        cmd_vel_timeout_sec_ = this->get_parameter("cmd_vel_timeout_sec").as_double();
        task_loop_hz_ = this->get_parameter("task_loop_hz").as_double();
        if (task_loop_hz_ <= 0.0) {
            task_loop_hz_ = 50.0;
        }
        task_loop_period_ = std::chrono::duration<double>(1.0 / task_loop_hz_);

        task_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/spider/task_cmd_vel", 10);
        context_ = std::make_shared<SpiderTaskContext>(this, task_cmd_vel_publisher_, cmd_vel_timeout_sec_);

        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/spider/cmd_vel",
            10,
            std::bind(&SpiderTaskNode::cmd_vel_callback, this, std::placeholders::_1));

        register_task(std::make_shared<StandTask>(context_.get(), "stand"));
        register_task(std::make_shared<MoveRequestTask>(context_.get(), "move_request"));
        init_task_manager("stand");

        if (auto task = get_task("stand")) {
            task->on_enter("");
        }

        task_thread_ = std::thread(&SpiderTaskNode::process_task_loop, this);

        RCLCPP_INFO(this->get_logger(), "spider_task_node started with explicit task scheduler");
        RCLCPP_INFO(this->get_logger(), "Listening to /spider/cmd_vel and forwarding /spider/task_cmd_vel");
        RCLCPP_INFO(this->get_logger(), "Initial task state = STAND");
    }

    ~SpiderTaskNode() override {
        shutdown_requested_.store(true);
        task_manager_cv_.notify_all();
        if (task_thread_.joinable()) {
            task_thread_.join();
        }
        if (context_) {
            context_->publish_zero_task_cmd_vel();
        }
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (context_) {
            context_->update_latest_cmd(*msg, this->now());
        }
        task_manager_cv_.notify_all();

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Received /spider/cmd_vel: vx=%.3f, vy=%.3f, wz=%.3f",
            msg->linear.x,
            msg->linear.y,
            msg->angular.z);
    }

    void register_task(std::shared_ptr<BaseTask> task_ptr) {
        if (!task_ptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to register task: task_ptr is null");
            return;
        }

        const std::string task_name = task_ptr->task_name_;
        if (task_name.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to register task: task name is empty");
            return;
        }

        task_table_[task_name] = std::move(task_ptr);
    }

    std::shared_ptr<BaseTask> get_task(const std::string& task_name) const {
        const auto it = task_table_.find(task_name);
        if (it == task_table_.end()) {
            return nullptr;
        }
        return it->second;
    }

    void init_task_manager(const std::string& first_task_name) {
        if (task_table_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No tasks registered, task scheduler will not start");
            return;
        }

        if (task_table_.find(first_task_name) == task_table_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Initial task [%s] is not registered", first_task_name.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(task_manager_mutex_);
        current_task_name_ = first_task_name;
        last_task_name_.clear();
        task_manager_initialized_ = true;
    }

    void process_task_loop() {
        while (rclcpp::ok() && !shutdown_requested_.load()) {
            std::shared_ptr<BaseTask> current_task;
            std::string current_task_name;
            std::string previous_task_name;

            {   
                std::unique_lock<std::mutex> lock(task_manager_mutex_);
                task_manager_cv_.wait_for(lock, task_loop_period_, [this]() {
                    return shutdown_requested_.load() || !rclcpp::ok();
                });

                if (!rclcpp::ok() || shutdown_requested_.load()) {
                    return;
                }

                if (!task_manager_initialized_ || current_task_name_.empty()) {
                    continue;
                }

                const auto task_it = task_table_.find(current_task_name_);
                if (task_it == task_table_.end()) {
                    RCLCPP_ERROR(this->get_logger(), "Current task [%s] is not registered", current_task_name_.c_str());
                    current_task_name_ = "stand";
                    continue;
                }

                current_task = task_it->second;
                current_task_name = current_task_name_;
                previous_task_name = last_task_name_;
            }

            std::string next_task_name;
            try {
                next_task_name = current_task->process(previous_task_name);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Task [%s] threw exception: %s",
                    current_task_name.c_str(),
                    e.what());
                if (context_) {
                    context_->publish_zero_task_cmd_vel();
                }
                next_task_name = "stand";
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Task [%s] threw an unknown exception", current_task_name.c_str());
                if (context_) {
                    context_->publish_zero_task_cmd_vel();
                }
                next_task_name = "stand";
            }

            if (next_task_name.empty()) {
                next_task_name = current_task_name;
            }

            std::shared_ptr<BaseTask> next_task;
            bool transitioned = false;
            {
                std::lock_guard<std::mutex> lock(task_manager_mutex_);
                last_task_name_ = current_task_name;

                auto next_it = task_table_.find(next_task_name);
                if (next_it == task_table_.end()) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Task [%s] requested unknown next task [%s], falling back to stand",
                        current_task_name.c_str(),
                        next_task_name.c_str());
                    next_task_name = "stand";
                    next_it = task_table_.find(next_task_name);
                }

                if (next_it == task_table_.end()) {
                    current_task_name_.clear();
                    continue;
                }

                transitioned = (next_task_name != current_task_name);
                current_task_name_ = next_task_name;
                next_task = next_it->second;
            }

            if (transitioned) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Task transition: [%s] -> [%s]",
                    current_task_name.c_str(),
                    next_task_name.c_str());
                next_task->on_enter(current_task_name);
            }
        }
    }

    std::shared_ptr<SpiderTaskContext> context_;
    std::map<std::string, std::shared_ptr<BaseTask>> task_table_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr task_cmd_vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    std::mutex task_manager_mutex_;
    std::condition_variable task_manager_cv_;
    bool task_manager_initialized_;
    std::string current_task_name_;
    std::string last_task_name_;
    std::thread task_thread_;
    std::atomic<bool> shutdown_requested_;
    double cmd_vel_timeout_sec_{0.25};
    double task_loop_hz_{50.0};
    std::chrono::duration<double> task_loop_period_{0.02};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpiderTaskNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
