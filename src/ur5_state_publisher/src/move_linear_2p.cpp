// Copyright (c) 2024 Emina Mahmutbegovic
//
// All rights reserved.

#include "joint_motion_generator.hpp"
#include "joint_state_publisher.hpp"

#include <cstdlib>
#include "control_msgs/JointTrajectoryControllerState.h"
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <ros/package.h>

const int joint_num = 6;
KDL::JntArray jnt_pos_start(joint_num);

void get_current_joint_angles(const control_msgs::JointTrajectoryControllerState::ConstPtr& ctr_msg) {
	jnt_pos_start(0) = ctr_msg->actual.positions[0];
    jnt_pos_start(1) = ctr_msg->actual.positions[1];
	jnt_pos_start(2) = ctr_msg->actual.positions[2];
	jnt_pos_start(3) = ctr_msg->actual.positions[3];
	jnt_pos_start(4) = ctr_msg->actual.positions[4];
	jnt_pos_start(5) = ctr_msg->actual.positions[5];
}

int main(int argc, char** argv) {
    std::string urdf_file_path = ros::package::getPath("ur5_gazebo");

	if(urdf_file_path.empty()) {
		ROS_ERROR("ur5_gazebo package path was not found");
	}
	urdf_file_path += "/urdf/ur5.xacro";

    // Init ROS
    ros::init(argc, argv, libs::ik_solver::kServiceName);

	// Parse urdf model and generate KDL tree
	KDL::Tree ur5_tree;
    if (!kdl_parser::treeFromFile(urdf_file_path, ur5_tree)){
        ROS_ERROR("Failed to construct KDL tree");
        return EXIT_FAILURE;
    }

    // Generate a kinematic chain from the robot base to its tcp
	KDL::Chain ur5_chain;
	ur5_tree.getChain(libs::ik_solver::ur5::kBaseLink, libs::ik_solver::ur5::kWrist3Link, ur5_chain);

    // Create Joint Motion Generator instance
    libs::ik_solver::JointMotionGenerator joint_motion_generator(ur5_chain);

    // Create Joint State Publisher instance
    libs::state_publisher::JointStatePublisher joint_state_publisher{};

    ros::NodeHandle node_handle;

    // Start spinner and wait
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    // Init publisher
    joint_state_publisher.init(node_handle);

    ros::Subscriber joint_angles_current = node_handle.subscribe(libs::state_publisher::ur5::topic::kControllerStateTopic, 
                                                                1000, 
                                                                get_current_joint_angles);
            
    ros::Rate loop_rate(joint_state_publisher.kLoopRate_());

    while (ros::ok()) {
        // Get user input
        std::cout << "Please define the first pose in which the motion should be completed:\n";

		double x, y, z, v, a;
		std::cout << " X: ";
		std::cin >> x;
		std::cout << " Y: ";
		std::cin >> y;
		std::cout << " Z: ";
		std::cin >> z;

        KDL::Vector desired_output_pose1(x, y, z);
        KDL::JntArray output_joint_angles1(6);
        joint_motion_generator.compute_joint_angles(desired_output_pose1, jnt_pos_start, output_joint_angles1);

        std::cout << "Please define the second pose, velocity, and acceleration in which the motion should be completed:\n";

		std::cout << " X: ";
		std::cin >> x;
		std::cout << " Y: ";
		std::cin >> y;
		std::cout << " Z: ";
		std::cin >> z;
        std::cout << " Velocity: ";
		std::cin >> v;
        std::cout << " Acceleration: ";
		std::cin >> a;

        KDL::Vector desired_output_pose2(x, y, z);
        KDL::JntArray output_joint_angles2(6);
        joint_motion_generator.compute_joint_angles(desired_output_pose2, jnt_pos_start, output_joint_angles2);

        // Populate output points
        std::vector<double> point1(6);
        for(auto i = 0; i < point1.size(); ++i) {
            point1[i] = output_joint_angles1(i);
        }

        std::vector<double> point2(6);
        for(auto i = 0; i < point2.size(); ++i) {
            point2[i] = output_joint_angles2(i);
        }

        joint_state_publisher.move_2p(point1, point2, v, a);
        
        // This will adjust as needed per iteration
        sleep(3);
    }

    ros::shutdown();
    
    return 0;
}
