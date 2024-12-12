#include "../include/joint_state_publisher.hpp"

#include <ros/ros.h>


int main(int argc, char** argv) {
    // Init ROS
    ros::init(argc, argv, ur5::state_publisher::kServiceName);

    // Create Joint State Publisher instance
    ur5::state_publisher::JointStatePublisher joint_state_publisher{};

    // Start spinner and wait
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    // Init publisher
    joint_state_publisher.init();
            
    ros::Rate loop_rate(joint_state_publisher.kLoopRate_());

   while (ros::ok()) {
        std::vector<double> positions = {0, -1.57, 0, -1.57, 0, 0};
        double velocity = 0;
        double acceleration = 0;

        joint_state_publisher.move_1p(positions, velocity, acceleration);
        
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    ros::shutdown();
    
    return 0;
}

