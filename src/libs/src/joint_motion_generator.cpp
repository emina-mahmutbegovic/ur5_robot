// Copyright (c) 2024 Emina Mahmutbegovic
//
// All rights reserved.

#include "joint_motion_generator.hpp"

#include <kdl/jntarray.hpp>
#include <ros/ros.h>

namespace libs::ik_solver {

void JointMotionGenerator::compute_joint_angles(
    KDL::Vector &pose, std::vector<double> &input_point,
    std::vector<double> &output_point) {

  // Compute current tcp position
  KDL::Frame input_tcp_pose;

  // Populate input joint angles
  KDL::JntArray input_joint_angles(input_point.size());
  for (auto i = 0; i < input_point.size(); ++i) {
    input_joint_angles(i) = input_point[i];
  }

  fk_solver_.JntToCart(input_joint_angles, input_tcp_pose);

  ROS_INFO("Current TCP Position/Twist KDL:");
  print_tcp_pose(input_tcp_pose);

  KDL::Frame output_tcp_pose(input_tcp_pose.M, pose);

  // Print for debugging purposes
  ROS_INFO("Output TCP Position/Twist KDL:");
  print_tcp_pose(output_tcp_pose);

  // Variable to store output values
  KDL::JntArray output_joint_angles(6);

  // Compute inverse kinematics
  ik_solver_.CartToJnt(input_joint_angles, output_tcp_pose,
                       output_joint_angles);

  // Populate output points
  for (auto i = 0; i < output_point.size(); ++i) {
    output_point[i] = output_joint_angles(i);
  }
}

void print_tcp_pose(const KDL::Frame &tcp_pose) {
  ROS_INFO("Position: %f %f %f", tcp_pose.p(0), tcp_pose.p(1), tcp_pose.p(2));
  ROS_INFO("Orientation: %f %f %f \n", tcp_pose.M(0, 0), tcp_pose.M(1, 0),
           tcp_pose.M(2, 0));
}

} // namespace libs::ik_solver
