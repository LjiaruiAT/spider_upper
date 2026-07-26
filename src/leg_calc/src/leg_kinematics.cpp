#include "leg_calc/leg_kinematics.hpp"

#include <algorithm>

namespace leg_calc {

LegKinematics::LegKinematics(const KDL::Chain& chain)
    : chain_(chain),
      fk_solver_(chain_),
      jacobian_solver_(chain_),
      velocity_solver_(chain_),
      ik_solver_(chain_, Eigen::Vector<double, 6>(1.0, 1.0, 1.0, 0.0, 0.0, 0.0), 1e-6, 150, 1e-10),
      jacobian_cache_(chain_.getNrOfJoints()),
      last_joint_solution_(chain_.getNrOfJoints()) {
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        last_joint_solution_(i) = 0.0;
    }
}

JointVector LegKinematics::inverse_position(const Eigen::Vector3d& foot_pos, int* result) {
    KDL::Frame target_frame;
    const Eigen::Vector3d target = foot_pos + position_offset_;
    target_frame.p = KDL::Vector(target.x(), target.y(), target.z());
    target_frame.M = KDL::Rotation::Identity();

    KDL::JntArray solution = last_joint_solution_;
    *result = ik_solver_.CartToJnt(last_joint_solution_, target_frame, solution);
    if (*result >= 0) {
        last_joint_solution_ = solution;
    }
    return from_kdl_joints(solution);
}

JointVector LegKinematics::inverse_velocity(const JointVector& joint_pos, const Eigen::Vector3d& foot_vel) {
    const KDL::JntArray joints = to_kdl_joints(joint_pos);
    jacobian_solver_.JntToJac(joints, jacobian_cache_);
    const JointMatrix jacobian = extract_position_jacobian(jacobian_cache_);
    return jacobian.completeOrthogonalDecomposition().solve(foot_vel);
}

Eigen::Vector3d LegKinematics::forward_position(const JointVector& joint_pos) {
    KDL::Frame frame;
    fk_solver_.JntToCart(to_kdl_joints(joint_pos), frame);
    return Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z()) - position_offset_;
}

Eigen::Vector3d LegKinematics::forward_velocity(const JointVector& joint_pos, const JointVector& joint_vel) {
    const KDL::JntArray joints = to_kdl_joints(joint_pos);
    jacobian_solver_.JntToJac(joints, jacobian_cache_);
    const JointMatrix jacobian = extract_position_jacobian(jacobian_cache_);
    return jacobian * joint_vel;
}

KDL::JntArray LegKinematics::to_kdl_joints(const JointVector& joints) {
    KDL::JntArray result(static_cast<unsigned int>(kLegJointDoF));
    for (unsigned int i = 0; i < kLegJointDoF; ++i) {
        result(i) = joints(static_cast<int>(i));
    }
    return result;
}

JointVector LegKinematics::from_kdl_joints(const KDL::JntArray& joints) {
    JointVector result = JointVector::Zero();
    const unsigned int count = std::min<unsigned int>(joints.rows(), static_cast<unsigned int>(kLegJointDoF));
    for (unsigned int i = 0; i < count; ++i) {
        result(static_cast<int>(i)) = joints(i);
    }
    return result;
}

JointMatrix LegKinematics::extract_position_jacobian(const KDL::Jacobian& full_jacobian) {
    JointMatrix jacobian = JointMatrix::Zero();
    for (int row = 0; row < static_cast<int>(kLegJointDoF); ++row) {
        for (int col = 0; col < static_cast<int>(kLegJointDoF); ++col) {
            jacobian(row, col) = full_jacobian(row, col);
        }
    }
    return jacobian;
}

}  // namespace leg_calc
