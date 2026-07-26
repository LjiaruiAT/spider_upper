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

class LegKinematics {
public:
    explicit LegKinematics(const KDL::Chain& chain);
    ~LegKinematics() = default;

    JointVector inverse_position(const Eigen::Vector3d& foot_pos, int* result);
    JointVector inverse_velocity(const JointVector& joint_pos, const Eigen::Vector3d& foot_vel);

    Eigen::Vector3d forward_position(const JointVector& joint_pos);
    Eigen::Vector3d forward_velocity(const JointVector& joint_pos, const JointVector& joint_vel);

    void set_position_offset(const Eigen::Vector3d& offset) { position_offset_ = offset; }
    const Eigen::Vector3d& position_offset() const { return position_offset_; }

private:
    static KDL::JntArray to_kdl_joints(const JointVector& joints);
    static JointVector from_kdl_joints(const KDL::JntArray& joints);
    static JointMatrix extract_position_jacobian(const KDL::Jacobian& full_jacobian);

    KDL::Chain chain_;
    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::ChainJntToJacSolver jacobian_solver_;
    KDL::ChainIkSolverVel_pinv velocity_solver_;
    KDL::ChainIkSolverPos_LMA ik_solver_;

    KDL::Jacobian jacobian_cache_;
    KDL::JntArray last_joint_solution_;

    Eigen::Vector3d position_offset_{Eigen::Vector3d::Zero()};
};

}  // namespace leg_calc
