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
    ros::init(argc, argv, ur5::ik_solver::kServiceName);

	// Parse urdf model and generate KDL tree
	KDL::Tree ur5_tree;
    if (!kdl_parser::treeFromFile(urdf_file_path, ur5_tree)){
        ROS_ERROR("Failed to construct KDL tree");
        return EXIT_FAILURE;
    }

    // Generate a kinematic chain from the robot base to its tcp
	KDL::Chain ur5_chain;
	ur5_tree.getChain(ur5::ik_solver::kBaseLink, ur5::ik_solver::kWrist3Link, ur5_chain);

    // Create Joint Motion Generator instance
    ur5::ik_solver::JointMotionGenerator joint_motion_generator(ur5_chain);

    // Create Joint State Publisher instance
    ur5::state_publisher::JointStatePublisher joint_state_publisher{};

    ros::NodeHandle node_handle;

    // Start spinner and wait
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    // Init publisher
    joint_state_publisher.init(node_handle);

    ros::Subscriber joint_angles_current = node_handle.subscribe(ur5::state_publisher::kControllerStateTopic, 1000, get_current_joint_angles);
            
    ros::Rate loop_rate(joint_state_publisher.kLoopRate_());

    while (ros::ok()) {
		KDL::JntArray output_joint_angles(6);

        // Get user input
        std::cout << "Please define the offset you want to move for each axis, velocity, and acceleration in which the motion should be completed:\n";

		double x, y, z, v, a;
		std::cout << " X:";
		std::cin >> x;
		std::cout << " Y:";
		std::cin >> y;
		std::cout << " Z:";
		std::cin >> z;
        std::cout << " Velocity:";
		std::cin >> v;
        std::cout << " Acceleration:";
		std::cin >> a;

        KDL::Vector desired_output_pose(x, y, z);

        joint_motion_generator.compute_joint_angles(desired_output_pose, jnt_pos_start, output_joint_angles);

        std::vector<double> positions(6);
        for(auto i = 0; i < positions.size(); ++i) {
            positions[i] = output_joint_angles(i);
        }

        joint_state_publisher.move_1p(positions, v, a);
        
        // This will adjust as needed per iteration
        sleep(2);
    }

    ros::shutdown();
    
    return 0;
}

