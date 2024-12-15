// Copyright (c) 2024 Emina Mahmutbegovic
//
// All rights reserved.

#include "joint_state_publisher.hpp"

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace libs::state_publisher::ur5 {

void load_trajectory_point(
    trajectory_msgs::JointTrajectoryPoint *joint_trajectory_point,
    const std::vector<double> &point, const double &velocity,
    const double &acceleration) {
  joint_trajectory_point->positions = point;

  joint_trajectory_point->velocities.resize(point.size());
  joint_trajectory_point->accelerations.resize(point.size());

  for (auto i = 0; i < point.size(); ++i) {
    joint_trajectory_point->velocities[i] = velocity;
    joint_trajectory_point->accelerations[i] = acceleration;
  }
}

void move_1p(const std::vector<double> &point, const double &velocity,
             const double &acceleration, ros::Publisher &joint_trajectory_pub) {

  ROS_INFO("Joint motion initiated:");
  ROS_INFO("Point: %f %f %f %f %f %f", point[0], point[1], point[2], point[3],
           point[4], point[5]);
  ROS_INFO("Velocity: %f", velocity);
  ROS_INFO("Acceleration: %f \n", acceleration);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory.joint_names = {
      ur5::joint::kShoulderPanJoint, ur5::joint::kShoulderLiftJoint,
      ur5::joint::kElbowJoint,       ur5::joint::kWrist1Joint,
      ur5::joint::kWrist2Joint,      ur5::joint::kWrist3Joint};

  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
  load_trajectory_point(&joint_trajectory_point, point, velocity, acceleration);

  joint_trajectory_point.time_from_start = ros::Duration(1.0);

  joint_trajectory.points.push_back(joint_trajectory_point);

  // Publish trajectory
  joint_trajectory_pub.publish(joint_trajectory);
}

void move_2p(const std::vector<double> &point1,
             const std::vector<double> &point2, const double &velocity,
             const double &acceleration, ros::Publisher &joint_trajectory_pub) {

  ROS_INFO("Trajectory motion initiated:");
  ROS_INFO("Point1: %f %f %f %f %f %f", point1[0], point1[1], point1[2],
           point1[3], point1[4], point1[5]);
  ROS_INFO("Point2: %f %f %f %f %f %f", point2[0], point2[1], point2[2],
           point2[3], point2[4], point2[5]);
  ROS_INFO("Velocity: %f", velocity);
  ROS_INFO("Acceleration: %f \n", acceleration);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory.joint_names = {
      ur5::joint::kShoulderPanJoint, ur5::joint::kShoulderLiftJoint,
      ur5::joint::kElbowJoint,       ur5::joint::kWrist1Joint,
      ur5::joint::kWrist2Joint,      ur5::joint::kWrist3Joint};

  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point1;
  ur5::load_trajectory_point(&joint_trajectory_point1, point1, velocity,
                             acceleration);

  joint_trajectory_point1.time_from_start = ros::Duration(1.0);

  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point2;
  ur5::load_trajectory_point(&joint_trajectory_point2, point2, velocity,
                             acceleration);

  joint_trajectory_point2.time_from_start = ros::Duration(2.0);

  joint_trajectory.points.resize(2);
  joint_trajectory.points[0] = joint_trajectory_point1;
  joint_trajectory.points[1] = joint_trajectory_point2;

  // Publish trajectory
  joint_trajectory_pub.publish(joint_trajectory);
}
} // namespace libs::state_publisher::ur5