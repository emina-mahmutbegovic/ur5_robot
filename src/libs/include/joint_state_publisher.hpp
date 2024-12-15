// Copyright (c) 2024 Emina Mahmutbegovic
//
// All rights reserved.

#ifndef JOINT_STATE_PUBLISHER_HPP
#define JOINT_STATE_PUBLISHER_HPP

#include <ros/ros.h>
#include <string>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <vector>

namespace libs::state_publisher {

namespace ur5 {

namespace topic {
const std::string kControllerCommandTopic =
    "/ur5/eff_joint_traj_controller/command";
const std::string kControllerStateTopic =
    "/ur5/eff_joint_traj_controller/state";
} // namespace topic

namespace joint {
const std::string kShoulderPanJoint = "shoulder_pan_joint";
const std::string kShoulderLiftJoint = "shoulder_lift_joint";
const std::string kElbowJoint = "elbow_joint";
const std::string kWrist1Joint = "wrist_1_joint";
const std::string kWrist2Joint = "wrist_2_joint";
const std::string kWrist3Joint = "wrist_3_joint";
} // namespace joint

void move_1p(const std::vector<double> &point, const double &velocity,
             const double &acceleration, ros::Publisher &joint_trajectory_pub);

void move_2p(const std::vector<double> &point1,
             const std::vector<double> &point2, const double &velocity,
             const double &acceleration, ros::Publisher &joint_trajectory_pub);

void load_trajectory_point(
    trajectory_msgs::JointTrajectoryPoint *joint_trajectory_point,
    const std::vector<double> &point, const double &velocity,
    const double &acceleration);

} // namespace ur5

// Define constants
const int kLoopRate = 50;
const int kQueueSize = 10;
const std::string kServiceName = "joint_state_publisher";

} // namespace libs::state_publisher

#endif // JOINT_STATE_PUBLISHER_HPP