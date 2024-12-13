// Copyright (c) 2024 Emina Mahmutbegovic
//
// All rights reserved.

#ifndef JOINT_MOTION_GENERATOR_HPP
#define JOINT_MOTION_GENERATOR_HPP

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <string>

namespace libs::ik_solver {

namespace ur5 {
    const std::string kBaseLink = "base_link";
    const std::string kWrist3Link = "wrist_3_link";
}

class JointMotionGenerator {

public:
    JointMotionGenerator() = default;
    JointMotionGenerator(const KDL::Chain &chain) 
        : fk_solver_(chain),
          vel_ik_solver_(chain, 0.0001, 1000),
          ik_solver_(chain, fk_solver_, vel_ik_solver_, 1000) {};

    JointMotionGenerator(const JointMotionGenerator &) = delete;
    JointMotionGenerator(JointMotionGenerator &&) = delete;

    JointMotionGenerator &operator=(const JointMotionGenerator &) = delete;
    JointMotionGenerator &operator=(JointMotionGenerator &&) = delete;

    ~JointMotionGenerator() = default;

    int init();

    void compute_joint_angles(KDL::Vector &pose, KDL::JntArray &input_joint_angles, KDL::JntArray &output_joint_angles);

private:
    void compute_output_pose(KDL::Vector &input_pose, KDL::Frame &input_tcp_pose, KDL::Vector *output_pose);

    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::ChainIkSolverVel_pinv vel_ik_solver_;
    KDL::ChainIkSolverPos_NR ik_solver_;
};

void print_tcp_pose(const KDL::Frame &tcp_pose);

// Define constants
const std::string kServiceName = "inverse_kinematics_solver";

} // namespace libs::ik_solver

#endif // JOINT_MOTION_GENERATOR_HPP