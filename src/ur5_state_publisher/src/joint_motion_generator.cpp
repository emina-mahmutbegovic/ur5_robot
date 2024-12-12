#include "joint_motion_generator.hpp"

#include <cstdlib>
#include <ros/ros.h>

namespace ur5 {
    
    int JointMotionGenerator::init() {
        // Parse urdf model and generate KDL tree
        if (!kdl_parser::treeFromFile(urdf_file_path_, ur5_tree_)){
            ROS_ERROR("Failed to construct kdl tree");
            return EXIT_FAILURE;
        }

        // Generate a kinematic chain from the robot base to its tcp
        ur5_tree_.getChain(kBaseLink, kWrist3Link, ur5_chain_);

        // fk_solver_ = new KDL::ChainFkSolverPos_recursive(ur5_chain_);
	    // vel_ik_solver_ = new KDL::ChainIkSolverVel_pinv(ur5_chain_, 0.0001, 1000);
	    // ik_solver_ = new KDL::ChainIkSolverPos_NR(ur5_chain_, fk_solver_, vel_ik_solver_, 1000);

        return EXIT_SUCCESS;
    }

} // namespace ur5
