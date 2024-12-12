#include "joint_state_publisher.hpp"

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace ur5::state_publisher {

    void JointStatePublisher::init() {
        joint_trajectory_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(kControllerCommandTopic, kQueueSize, true);
    }

    void JointStatePublisher::load_trajectory_point(trajectory_msgs::JointTrajectoryPoint *joint_trajectory_point, 
                                const std::vector<double> &point, 
                                const double &velocity,
                                const double &acceleration) {
        joint_trajectory_point->positions = point;
      
        joint_trajectory_point->velocities.resize(6);
        joint_trajectory_point->accelerations.resize(6);

        for(auto i = 0; i < point.size(); ++i) {
            joint_trajectory_point->velocities[i] = velocity;
            joint_trajectory_point->accelerations[i] = acceleration;
        }
    }

    void JointStatePublisher::move_1p(const std::vector<double> &point, 
                                      const double &velocity,
                                      const double &acceleration) {

        trajectory_msgs::JointTrajectory joint_trajectory;
        joint_trajectory.header.stamp = ros::Time::now();
        joint_trajectory.joint_names = {kShoulderPanJoint, kShoulderLiftJoint, kElbowJoint, kWrist1Joint, kWrist2Joint, kWrist3Joint};

        trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
        load_trajectory_point(&joint_trajectory_point, point, velocity, acceleration);

        joint_trajectory_point.time_from_start = ros::Duration(1.0);

        joint_trajectory.points.push_back(joint_trajectory_point);

        // Publisj trajectory
        joint_trajectory_pub_.publish(joint_trajectory);
    }

    void JointStatePublisher::move_2p(const std::vector<double> &point1, 
                                    const std::vector<double> &point2,
                                    const double &velocity,
                                    const double &acceleration) {

        trajectory_msgs::JointTrajectory joint_trajectory;
        joint_trajectory.header.stamp = ros::Time::now();
        joint_trajectory.joint_names = {kShoulderPanJoint, kShoulderLiftJoint, kElbowJoint, kWrist1Joint, kWrist2Joint, kWrist3Joint};

        trajectory_msgs::JointTrajectoryPoint joint_trajectory_point1;
        load_trajectory_point(&joint_trajectory_point1, point1, velocity, acceleration);

        joint_trajectory_point1.time_from_start = ros::Duration(1.0);

        trajectory_msgs::JointTrajectoryPoint joint_trajectory_point2;
        load_trajectory_point(&joint_trajectory_point2, point2, velocity, acceleration);

        joint_trajectory_point2.time_from_start = ros::Duration(2.0);

        joint_trajectory.points.resize(2);
        joint_trajectory.points[0] = joint_trajectory_point1;
        joint_trajectory.points[1] = joint_trajectory_point2;

        // Publish trajectory
        joint_trajectory_pub_.publish(joint_trajectory);
    }
}