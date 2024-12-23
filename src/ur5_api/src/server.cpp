// Copyright (c) 2024 Emina Mahmutbegovic
//
// All rights reserved.

#include <numeric>
#include <tf/transform_listener.h>
#include <vector>

#include "joint_state_publisher.hpp"
#include "server.hpp"

namespace ur5::server {

control_msgs::JointTrajectoryControllerState curr_joint_state;

UR5Server::UR5Server(ros::NodeHandle &node_handle, KDL::Chain &chain)
    : joint_motion_generator_(chain) {
  joint_trajectory_pub_ =
      node_handle.advertise<trajectory_msgs::JointTrajectory>(
          libs::state_publisher::ur5::topic::kControllerCommandTopic,
          libs::state_publisher::kQueueSize, true);

  read_joint_state_service_ = node_handle.advertiseService(
      kReadJointRobotStateService, read_joint_robot_state);

  read_transform_state_service_ = node_handle.advertiseService(
      kReadTransformRobotStateService, read_transform_robot_state);

  move_joint_service_ = node_handle.advertiseService(
      kMoveJointService, &UR5Server::move_joint, this);

  move_linear_service_ = node_handle.advertiseService(
      kMoveLinearService, &UR5Server::move_linear, this);
}

bool UR5Server::move_joint(ur5_api::MoveJoint::Request &req,
                           ur5_api::MoveJoint::Response &res) {
  ROS_INFO("Received MoveJoint request:");

  ROS_INFO("Point1: [ %s ]",
           std::accumulate(req.point1.begin(), req.point1.end(), std::string(),
                           [](const std::string &acc, double value) {
                             return acc + std::to_string(value) + " ";
                           })
               .c_str());
  ROS_INFO("Point2: [ %s ]",
           std::accumulate(req.point2.begin(), req.point2.end(), std::string(),
                           [](const std::string &acc, double value) {
                             return acc + std::to_string(value) + " ";
                           })
               .c_str());
  ROS_INFO("Velocity: %f, Acceleration: %f", req.velocity, req.acceleration);

  try {
    if (req.point1.size() != req.point2.size()) {
      ROS_ERROR("Point1 and Point2 must have the same size!");
      return false; // Indicate failure
    }

    // Generate movement
    libs::state_publisher::ur5::move_2p(req.point1, req.point2, req.velocity,
                                        req.acceleration,
                                        joint_trajectory_pub_);

    // Set response
    set_joint_state_response(res.joint_state);

    // Log success
    ROS_INFO("MoveJoint operation completed successfully.");

    return true;

  } catch (const std::exception &e) {
    ROS_ERROR("Exception during MoveJoint processing: %s", e.what());

    return false;
  }
}

bool UR5Server::move_linear(ur5_api::MoveLinear::Request &req,
                            ur5_api::MoveLinear::Response &res) {
  ROS_INFO("Received MoveLinear request:");

  ROS_INFO("Pose1: [ %s ]",
           std::accumulate(req.pose1.begin(), req.pose1.end(), std::string(),
                           [](const std::string &acc, double value) {
                             return acc + std::to_string(value) + " ";
                           })
               .c_str());
  ROS_INFO("Pose2: [ %s ]",
           std::accumulate(req.pose2.begin(), req.pose2.end(), std::string(),
                           [](const std::string &acc, double value) {
                             return acc + std::to_string(value) + " ";
                           })
               .c_str());
  ROS_INFO("Velocity: %f, Acceleration: %f", req.velocity, req.acceleration);

  try {
    if (req.pose1.size() != req.pose2.size()) {
      ROS_ERROR("Pose1 and Pose2 must have the same size!");
      return false; // Indicate failure
    }

    // Extract desired output pose1
    KDL::Vector desired_output_pose1(req.pose1[0], req.pose1[1], req.pose1[2]);

    // Calculate output poin1
    std::vector<double> point1(6);
    joint_motion_generator_.compute_joint_angles(
        desired_output_pose1, curr_joint_state.actual.positions, point1);

    // Generate movement
    libs::state_publisher::ur5::move_1p(point1, req.velocity, req.acceleration,
                                        joint_trajectory_pub_);

    sleep(1.0);

    // Get desired output pose2
    KDL::Vector desired_output_pose2(req.pose2[0], req.pose2[1], req.pose2[2]);

    // Calculate output point2
    std::vector<double> point2(6);
    joint_motion_generator_.compute_joint_angles(
        desired_output_pose2, curr_joint_state.actual.positions, point2);

    // Generate movement
    libs::state_publisher::ur5::move_1p(point2, req.velocity, req.acceleration,
                                        joint_trajectory_pub_);

    // Set response
    set_transform_state_response(res.transform.transforms);

    // Log success
    ROS_INFO("MoveLinear operation completed successfully.");

    return true;
  } catch (const std::exception &e) {
    ROS_ERROR("Exception during MoveLinear processing: %s", e.what());

    return false;
  }
}

void get_current_joint_trajectory_ctr_state(
    const control_msgs::JointTrajectoryControllerState::ConstPtr &ctr_msg) {
  // Set current trajectory controller state

  // Header
  curr_joint_state.header.stamp = ctr_msg->header.stamp;
  curr_joint_state.header.frame_id = ctr_msg->header.frame_id;

  // Joint names
  curr_joint_state.joint_names = ctr_msg->joint_names;

  // Actual state
  curr_joint_state.actual.positions = ctr_msg->actual.positions;
  curr_joint_state.actual.accelerations = ctr_msg->actual.accelerations;
  curr_joint_state.actual.velocities = ctr_msg->actual.velocities;

  // Desired state
  curr_joint_state.desired.positions = ctr_msg->desired.positions;
  curr_joint_state.desired.accelerations = ctr_msg->desired.accelerations;
  curr_joint_state.desired.velocities = ctr_msg->desired.velocities;

  // Error
  curr_joint_state.error.positions = ctr_msg->error.positions;
  curr_joint_state.error.accelerations = ctr_msg->error.accelerations;
  curr_joint_state.error.velocities = ctr_msg->error.velocities;
}

// Read joint robot state service callback
bool read_joint_robot_state(ur5_api::ReadRobotState::Request &req,
                            ur5_api::ReadRobotState::Response &res) {

  ROS_INFO("Read joint state request received.");

  // Set response
  set_joint_state_response(res.joint_state);

  return true;
}

// Read transform robot state service callback
bool read_transform_robot_state(ur5_api::ReadRobotState::Request &req,
                                ur5_api::ReadRobotState::Response &res) {

  ROS_INFO("Read transform state request received.");

  // Set response
  set_transform_state_response(res.transform.transforms);

  return true;
}

void set_joint_state_response(
    control_msgs::JointTrajectoryControllerState &state) {
  // Header
  state.header.stamp = curr_joint_state.header.stamp;
  state.header.frame_id = curr_joint_state.header.frame_id;

  // Joint names
  state.joint_names = curr_joint_state.joint_names;

  // Actual state
  state.actual.positions = curr_joint_state.actual.positions;
  state.actual.accelerations = curr_joint_state.actual.accelerations;
  state.actual.velocities = curr_joint_state.actual.velocities;

  // Desired state
  state.desired.positions = curr_joint_state.desired.positions;
  state.desired.accelerations = curr_joint_state.desired.accelerations;
  state.desired.velocities = curr_joint_state.desired.velocities;

  // Error
  state.error.positions = curr_joint_state.error.positions;
  state.error.accelerations = curr_joint_state.error.accelerations;
  state.error.velocities = curr_joint_state.error.velocities;
}

void set_transform_state_response(
    std::vector<geometry_msgs::TransformStamped> &transforms) {
  // TF Buffer and Listener
  tf::TransformListener listener;
  tf::StampedTransform transform;

  listener.waitForTransform(kBaseLink, kTool0, ros::Time(0),
                            ros::Duration(2.0));
  listener.lookupTransform(kBaseLink, kTool0, ros::Time(0), transform);

  transforms.resize(1);

  // Update translations
  transforms[0].transform.translation.x = transform.getOrigin().x();
  transforms[0].transform.translation.y = transform.getOrigin().y();
  transforms[0].transform.translation.z = transform.getOrigin().z();

  // Update rotations
  transforms[0].transform.rotation.x = transform.getRotation().x();
  transforms[0].transform.rotation.y = transform.getRotation().y();
  transforms[0].transform.rotation.z = transform.getRotation().z();
  transforms[0].transform.rotation.w = transform.getRotation().w();
}

} // namespace ur5::server
