// Copyright (c) 2024 Emina Mahmutbegovic
//
// All rights reserved.

#include "joint_motion_generator.hpp"

#include <ros/ros.h>

namespace libs::ik_solver {
    
    void JointMotionGenerator::compute_joint_angles(KDL::Vector &pose, 
                                                    KDL::JntArray &input_joint_angles, 
                                                    KDL::JntArray &output_joint_angles) {
        // Compute current tcp position
		KDL::Frame input_tcp_pose;
		fk_solver_.JntToCart(input_joint_angles, input_tcp_pose);

		ROS_INFO("Current TCP Position/Twist KDL:");		
		print_tcp_pose(input_tcp_pose);

        KDL::Vector output_pose(0.0, 0.0, 0.0);
        for(auto i = 0; i < 3; ++i) {
            output_pose(i) = input_tcp_pose.p(i) + pose(i);
            if(output_pose(i) > 1) {
                output_pose(i) = output_pose(i) - 1;
            }
        }

		KDL::Frame output_tcp_pose(input_tcp_pose.M, output_pose);

        ROS_INFO("Output TCP Position/Twist KDL:");		
		print_tcp_pose(output_tcp_pose);

		// Compute inverse kinematics
		ik_solver_.CartToJnt(input_joint_angles, output_tcp_pose, output_joint_angles);
    }

    void print_tcp_pose(const KDL::Frame &tcp_pose) {
        ROS_INFO("Position: %f %f %f", tcp_pose.p(0), tcp_pose.p(1), tcp_pose.p(2));		
		ROS_INFO("Orientation: %f %f %f \n", tcp_pose.M(0,0), tcp_pose.M(1,0), tcp_pose.M(2,0));
    }

} // namespace libs::ik_solver
