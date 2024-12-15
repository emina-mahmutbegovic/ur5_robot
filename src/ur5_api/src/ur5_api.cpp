#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "joint_state_publisher.hpp"
#include "server.hpp"
#include <ros/package.h>

static ros::Publisher joint_trajectory_pub;

int main(int argc, char **argv) {
  ros::init(argc, argv, ur5::server::kServiceName);

  std::string urdf_file_path = ros::package::getPath("ur5_gazebo");

  if (urdf_file_path.empty()) {
    ROS_ERROR("ur5_gazebo package path was not found");
  }
  urdf_file_path += "/urdf/ur5.xacro";

  // Parse urdf model and generate KDL tree
  KDL::Tree ur5_tree;
  if (!kdl_parser::treeFromFile(urdf_file_path, ur5_tree)) {
    ROS_ERROR("Failed to construct KDL tree");
    return EXIT_FAILURE;
  }

  // Generate a kinematic chain from the robot base to its tcp
  KDL::Chain ur5_chain;
  ur5_tree.getChain(libs::ik_solver::ur5::kBaseLink,
                    libs::ik_solver::ur5::kWrist3Link, ur5_chain);

  ros::NodeHandle node_handle;

  ur5::server::UR5Server server(node_handle, ur5_chain);

  // Subscribe to current states
  // TODO Move this to server
  ros::Subscriber joint_angles_current = node_handle.subscribe(
      libs::state_publisher::ur5::topic::kControllerStateTopic, 1000,
      ur5::server::get_current_joint_trajectory_ctr_state);

  ROS_INFO("UR5 Server Ready...");
  ros::spin();

  return 0;
}
