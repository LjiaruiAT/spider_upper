#pragma once

#include "leg_calc/common_types.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace leg_calc {

// 单腿运动学封装：把“足端位置/速度”和“关节角/角速度”互相转换。
//
// 这里的 chain 决定了真正的几何关系 f(q)。因此，IK 算法本身并不认识
// “蜘蛛腿”这个概念；如果 KDL 链的杆长、关节轴或坐标系定义不对，求解器
// 仍可能正常返回，但结果不会对应真实机械结构。
class LegKinematics {
public:
    explicit LegKinematics(const KDL::Chain& chain);
    ~LegKinematics() = default;

    // 位置逆运动学：给定腿坐标系下的足端目标 p，求 q，使 f(q) 尽量接近 p。
    // result 是 KDL 返回码；非负通常表示求解成功，失败时返回解不应直接视为可靠解。
    JointVector inverse_position(const Eigen::Vector3d& foot_pos, int* result);

    // 速度逆运动学：给定当前 q 和足端线速度 v，求 q_dot，使 J(q) q_dot ≈ v。
    JointVector inverse_velocity(const JointVector& joint_pos, const Eigen::Vector3d& foot_vel);

    // 正运动学：给定 q，计算足端位置 p=f(q)。
    Eigen::Vector3d forward_position(const JointVector& joint_pos);

    // 速度正运动学：给定 q 和 q_dot，计算足端线速度 v=J(q)q_dot。
    Eigen::Vector3d forward_velocity(const JointVector& joint_pos, const JointVector& joint_vel);

    // 给 KDL 链的内部坐标增加一个位置偏移；默认值为零。
    // IK 会把 API 输入加上该偏移，FK 返回 API 坐标时再减回去。
    void set_position_offset(const Eigen::Vector3d& offset) { position_offset_ = offset; }
    const Eigen::Vector3d& position_offset() const { return position_offset_; }

private:
    static KDL::JntArray to_kdl_joints(const JointVector& joints);
    static JointVector from_kdl_joints(const KDL::JntArray& joints);
    static JointMatrix extract_position_jacobian(const KDL::Jacobian& full_jacobian);

    KDL::Chain chain_;
    // 三类 solver 分别负责位置 FK、Jacobian、位置/速度逆解；cache 避免重复申请 Jacobian 容器。
    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::ChainJntToJacSolver jacobian_solver_;
    KDL::ChainIkSolverVel_pinv velocity_solver_;
    KDL::ChainIkSolverPos_LMA ik_solver_;

    KDL::Jacobian jacobian_cache_;
    KDL::JntArray last_joint_solution_;

    Eigen::Vector3d position_offset_{Eigen::Vector3d::Zero()};
};

}  // namespace leg_calc
