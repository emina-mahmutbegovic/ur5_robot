#include "../include/joint_state_publisher.hpp"

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace ur5::state_publisher {

    void JointStatePublisher::init() {
        joint_trajectory_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(kControllerCommandTopic, kQueueSize, true);
    }

    void JointStatePublisher::update(const std::vector<double> &positions, 
                                    const std::vector<double> &velocities,
                                    const std::vector<double> &accelerations) {

        trajectory_msgs::JointTrajectory joint_trajectory;
        joint_trajectory.header.stamp = ros::Time::now();
        joint_trajectory.joint_names = {kShoulderPanJoint, kShoulderLiftJoint, kElbowJoint, kWrist1Joint, kWrist2Joint, kWrist3Joint};

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = positions;
        point.velocities = velocities;
        point.accelerations = accelerations;
        point.time_from_start = ros::Duration(1.0);

        joint_trajectory.points.push_back(point);

        // Send the joint state and transform
        joint_trajectory_pub_.publish(joint_trajectory);
    }

    void JointStatePublisher::move(const std::vector<double> &point1, 
                                    const std::vector<double> &point2,
                                    const double &velocity,
                                    const double &accelleration) {

        trajectory_msgs::JointTrajectory joint_trajectory;
        joint_trajectory.header.stamp = ros::Time::now();
        joint_trajectory.joint_names = {kShoulderPanJoint, kShoulderLiftJoint, kElbowJoint, kWrist1Joint, kWrist2Joint, kWrist3Joint};

        trajectory_msgs::JointTrajectoryPoint joint_trajectory_point1;
        joint_trajectory_point1.positions = point1;

        for(auto i = 0; i < point1.size(); ++i) {
            joint_trajectory_point1.velocities[i] = velocity;
            joint_trajectory_point1.accelerations[i] = accelleration;
        }
        joint_trajectory_point1.time_from_start = ros::Duration(1.0);

        trajectory_msgs::JointTrajectoryPoint joint_trajectory_point2;
        joint_trajectory_point2.positions = point2;

        for(auto i = 0; i < point2.size(); ++i) {
            joint_trajectory_point2.velocities[i] = velocity;
            joint_trajectory_point2.accelerations[i] = accelleration;
        }

        joint_trajectory_point2.time_from_start = ros::Duration(2.0);

        joint_trajectory.points.resize(2);
        joint_trajectory.points[0] = joint_trajectory_point1;
        joint_trajectory.points[1] = joint_trajectory_point2;

        // Send the joint state and transform
        joint_trajectory_pub_.publish(joint_trajectory);
    }
}