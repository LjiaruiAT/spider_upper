#include "leg_calc/leg_kinematics.hpp"

#include <algorithm>

namespace leg_calc {

// KDL 链是这套运动学的“几何真相”：它包含关节轴、杆件长度和末端固定变换。
// 所有 FK/IK/Jacobian 计算都围绕同一条 chain_ 进行，必须保证它与真实腿的
// 坐标系、关节顺序和尺寸一致；否则求解器收敛也不代表机械结果正确。
LegKinematics::LegKinematics(const KDL::Chain& chain)
    : chain_(chain),
      fk_solver_(chain_),
      jacobian_solver_(chain_),
      velocity_solver_(chain_),
      // KDL 的 6 个任务维度为 [x, y, z, rotation_x, rotation_y, rotation_z]。
      // 这里权重为 [1,1,1,0,0,0]，表示只要求位置，不约束末端姿态。
      // 1e-6 是 LMA 的误差收敛阈值，150 是最大迭代次数，1e-10 是关节增量收敛阈值。
      ik_solver_(chain_, Eigen::Vector<double, 6>(1.0, 1.0, 1.0, 0.0, 0.0, 0.0), 1e-6, 150, 1e-10),
      jacobian_cache_(chain_.getNrOfJoints()),
      last_joint_solution_(chain_.getNrOfJoints()) {
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        last_joint_solution_(i) = 0.0;
    }
}

JointVector LegKinematics::inverse_position(const Eigen::Vector3d& foot_pos, int* result) {
    // API 接收的是腿局部坐标系下的目标点。position_offset_ 用于处理 KDL 链原点
    // 与 API 原点之间的固定平移差异；当前默认为零，但必须在 IK/FK 两边成对使用。
    KDL::Frame target_frame;
    const Eigen::Vector3d target = foot_pos + position_offset_;
    target_frame.p = KDL::Vector(target.x(), target.y(), target.z());
    // 位置 IK 不要求姿态，因此给一个单位旋转作为“被忽略的姿态部分”。
    target_frame.M = KDL::Rotation::Identity();

    // 位置 IK 的本质是求 q，使 f(q) ≈ target。
    // 上一周期的解作为初值可以减少连续轨迹中的跳变，也更容易保持同一分支。
    KDL::JntArray solution = last_joint_solution_;
    *result = ik_solver_.CartToJnt(last_joint_solution_, target_frame, solution);
    if (*result >= 0) {
        // 只有 KDL 报告成功才更新暖启动缓存；失败时保留上一个可信解。
        last_joint_solution_ = solution;
    }
    // 注意：这里仍然返回 KDL 填写的 solution；调用方必须结合 result 判断它是否可信。
    return from_kdl_joints(solution);
}

JointVector LegKinematics::inverse_velocity(const JointVector& joint_pos, const Eigen::Vector3d& foot_vel) {
    // 速度 IK 使用当前姿态处的 Jacobian：
    //   v = J(q) * q_dot
    // 已知目标足端线速度 v，反过来求关节角速度 q_dot。
    const KDL::JntArray joints = to_kdl_joints(joint_pos);
    jacobian_solver_.JntToJac(joints, jacobian_cache_);
    const JointMatrix jacobian = extract_position_jacobian(jacobian_cache_);

    // 当前用 Eigen 的完全正交分解求最小二乘意义下的解 J*q_dot ≈ foot_vel。
    // 当目标不可精确实现或 Jacobian 接近奇异时，这比直接求逆更稳健；但这里还没有
    // 额外的速度限幅、阻尼或奇异性报警。
    return jacobian.completeOrthogonalDecomposition().solve(foot_vel);
}

Eigen::Vector3d LegKinematics::forward_position(const JointVector& joint_pos) {
    // FK 是 IK 的反方向：给定 q，通过同一条 KDL chain 计算足端 p=f(q)。
    KDL::Frame frame;
    fk_solver_.JntToCart(to_kdl_joints(joint_pos), frame);
    // 撤销与 inverse_position 对称加入的 offset，让 FK 返回值仍处在 API 坐标约定下。
    return Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z()) - position_offset_;
}

Eigen::Vector3d LegKinematics::forward_velocity(const JointVector& joint_pos, const JointVector& joint_vel) {
    // 速度 FK：先在 q 处计算 Jacobian，再做矩阵乘法 v=J(q)q_dot。
    const KDL::JntArray joints = to_kdl_joints(joint_pos);
    jacobian_solver_.JntToJac(joints, jacobian_cache_);
    const JointMatrix jacobian = extract_position_jacobian(jacobian_cache_);
    return jacobian * joint_vel;
}

KDL::JntArray LegKinematics::to_kdl_joints(const JointVector& joints) {
    // Eigen 和 KDL 使用不同的容器；这里只做 [coxa, femur, tibia] 的顺序保持转换。
    KDL::JntArray result(static_cast<unsigned int>(kLegJointDoF));
    for (unsigned int i = 0; i < kLegJointDoF; ++i) {
        result(i) = joints(static_cast<int>(i));
    }
    return result;
}

JointVector LegKinematics::from_kdl_joints(const KDL::JntArray& joints) {
    // 返回固定大小的 3 DoF 向量；若 KDL 链更长，只取前三个关节，若更短则余下保持零。
    JointVector result = JointVector::Zero();
    const unsigned int count = std::min<unsigned int>(joints.rows(), static_cast<unsigned int>(kLegJointDoF));
    for (unsigned int i = 0; i < count; ++i) {
        result(static_cast<int>(i)) = joints(i);
    }
    return result;
}

JointMatrix LegKinematics::extract_position_jacobian(const KDL::Jacobian& full_jacobian) {
    // KDL Jacobian 的前 3 行是线速度部分 [vx, vy, vz]，后 3 行是角速度部分。
    // 当前的位置任务只需要前 3 行，形成 J_position，使 v = J_position*q_dot。
    JointMatrix jacobian = JointMatrix::Zero();
    for (int row = 0; row < static_cast<int>(kLegJointDoF); ++row) {
        for (int col = 0; col < static_cast<int>(kLegJointDoF); ++col) {
            jacobian(row, col) = full_jacobian(row, col);
        }
    }
    return jacobian;
}

}  // namespace leg_calc
