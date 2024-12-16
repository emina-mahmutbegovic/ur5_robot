// Copyright (c) 2024 Emina Mahmutbegovic
//
// All rights reserved.

#ifndef SERVER_HPP
#define SERVER_HPP

#include "control_msgs/JointTrajectoryControllerState.h"
#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <kdl/chain.hpp>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

#include "joint_motion_generator.hpp"
#include "ur5_api/MoveJoint.h"
#include "ur5_api/MoveLinear.h"
#include "ur5_api/ReadRobotState.h"

namespace ur5::server {

class UR5Server {
public:
  UR5Server() = default;
  UR5Server(ros::NodeHandle &node_handle, KDL::Chain &chain);

  UR5Server(const UR5Server &) = delete;
  UR5Server(UR5Server &&) = delete;

  UR5Server &operator=(const UR5Server &) = delete;
  UR5Server &operator=(UR5Server &&) = delete;

  ~UR5Server() = default;

  // Callback handlers
  bool move_joint(ur5_api::MoveJoint::Request &req,
                  ur5_api::MoveJoint::Response &res);
  bool move_linear(ur5_api::MoveLinear::Request &req,
                   ur5_api::MoveLinear::Response &res);

private:
  // Publisher
  ros::Publisher joint_trajectory_pub_;

  // Services
  ros::ServiceServer read_joint_state_service_;
  ros::ServiceServer read_transform_state_service_;
  ros::ServiceServer move_joint_service_;
  ros::ServiceServer move_linear_service_;

  // Joint motion generator
  libs::ik_solver::JointMotionGenerator joint_motion_generator_;
};

// Callback handlers
bool read_joint_robot_state(ur5_api::ReadRobotState::Request &req,
                            ur5_api::ReadRobotState::Response &res);
bool read_transform_robot_state(ur5_api::ReadRobotState::Request &req,
                                ur5_api::ReadRobotState::Response &res);

// Getters/setters
void get_current_joint_trajectory_ctr_state(
    const control_msgs::JointTrajectoryControllerState::ConstPtr &ctr_msg);

void set_joint_state_response(
    control_msgs::JointTrajectoryControllerState &state);

void set_transform_state_response(
    std::vector<geometry_msgs::TransformStamped> &transforms);

const std::string kBaseLink = "base_link";
const std::string kServiceName = "ur5_server";
const std::string kReadJointRobotStateService = "read_joint_robot_state";
const std::string kReadTransformRobotStateService =
    "read_transform_robot_state";
const std::string kMoveJointService = "move_joint";
const std::string kMoveLinearService = "move_linear";
const std::string kTool0 = "tool0";

} // namespace ur5::server

#endif // SERVER_HPP