#include "joint_state_publisher.hpp"

#include <ros/ros.h>


int main(int argc, char** argv) {
    // Init ROS
    ros::init(argc, argv, ur5::state_publisher::kServiceName);

    // Create Joint State Publisher instance
    ur5::state_publisher::JointStatePublisher joint_state_publisher{};

    ros::NodeHandle node_handle;

    // Start spinner and wait
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    // Init publisher
    joint_state_publisher.init(node_handle);
            
    ros::Rate loop_rate(joint_state_publisher.kLoopRate_());

   while (ros::ok()) {
        // Get user input
        std::cout << "Please define the angles, velocity, and acceleration in which the motion should be completed:\n";

		double sh_pan_jnt, sh_lift_jnt, ebw_jnt, wr_1_jnt, wr_2_jnt, wr_3_jnt, v, a;
		std::cout << " Shoulder Pan Joint:";
		std::cin >> sh_pan_jnt;
		std::cout << " Shoulder Lift Joint:";
		std::cin >> sh_lift_jnt;
		std::cout << " Elbow Joint:";
		std::cin >> ebw_jnt;
        std::cout << " Wrist 1 Joint:";
		std::cin >> wr_1_jnt;
        std::cout << " Wrist 2 Joint:";
		std::cin >> wr_2_jnt;
        std::cout << " Wrist 3 Joint:";
		std::cin >> wr_3_jnt;
        std::cout << " Velocity:";
		std::cin >> v;
        std::cout << " Acceleration:";
		std::cin >> a;

        std::vector<double> positions = {sh_pan_jnt, sh_lift_jnt, ebw_jnt, wr_1_jnt, wr_2_jnt, wr_3_jnt};

        joint_state_publisher.move_1p(positions, v, a);
        
        // This will adjust as needed per iteration
        sleep(2);
    }

    ros::shutdown();
    
    return 0;
}

