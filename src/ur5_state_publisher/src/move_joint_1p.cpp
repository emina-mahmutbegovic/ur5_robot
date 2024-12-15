#include "joint_state_publisher.hpp"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace libs::state_publisher;

int main(int argc, char **argv) {
  // Init ROS
  ros::init(argc, argv, kServiceName);

  ros::NodeHandle node_handle;

  // Start spinner and wait
  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(2.0);

  // Init publisher
  ros::Publisher joint_trajectory_pub =
      node_handle.advertise<trajectory_msgs::JointTrajectory>(
          ur5::topic::kControllerCommandTopic, kQueueSize, true);

  ros::Rate loop_rate(kLoopRate);

  while (ros::ok()) {
    // Get user input
    std::cout << "Please define the angles, velocity, and acceleration in "
                 "which the motion should be completed:\n";

    double sh_pan_jnt, sh_lift_jnt, ebw_jnt, wr_1_jnt, wr_2_jnt, wr_3_jnt, v, a;
    std::cout << " Shoulder Pan Joint: ";
    std::cin >> sh_pan_jnt;
    std::cout << " Shoulder Lift Joint: ";
    std::cin >> sh_lift_jnt;
    std::cout << " Elbow Joint: ";
    std::cin >> ebw_jnt;
    std::cout << " Wrist 1 Joint: ";
    std::cin >> wr_1_jnt;
    std::cout << " Wrist 2 Joint: ";
    std::cin >> wr_2_jnt;
    std::cout << " Wrist 3 Joint: ";
    std::cin >> wr_3_jnt;
    std::cout << " Velocity: ";
    std::cin >> v;
    std::cout << " Acceleration: ";
    std::cin >> a;

    std::vector<double> positions = {sh_pan_jnt, sh_lift_jnt, ebw_jnt,
                                     wr_1_jnt,   wr_2_jnt,    wr_3_jnt};

    ur5::move_1p(positions, v, a, joint_trajectory_pub);

    // This will adjust as needed per iteration
    sleep(2);
  }

  ros::shutdown();

  return 0;
}
