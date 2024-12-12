#include <string>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>



// Define joint names
const std::string shoulder_pan_joint = "shoulder_pan_joint";
const std::string shoulder_lift_joint = "shoulder_lift_joint";
const std::string elbow_joint = "elbow_joint";
const std::string wrist_1_joint = "wrist_1_joint";
const std::string wrist_2_joint = "wrist_2_joint";
const std::string wrist_3_joint = "wrist_3_joint";

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);
        
    // Create publisher for joint trajectory
    ros::Publisher joint_trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ur5/eff_joint_traj_controller/command", 1000, true);
	
    //Create subscriber for joint trajectory
	ros::Subscriber joint_trajectory_sub = n.subscribe("/ur5/eff_joint_traj_controller/state", 1000, get_shoulder_pan_joint_position);
    
    ros::Rate loop_rate(30);

   while (ros::ok()) {

        trajectory_msgs::JointTrajectory joint_trajectory;
        joint_trajectory.header.stamp = ros::Time::now();
        joint_trajectory.joint_names = {shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint};

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0, -1.57, 0, -1.57, 0, 0};
        point.velocities = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        point.time_from_start = ros::Duration(2.0);

        joint_trajectory.points.push_back(point);

        //send the joint state and transform
        joint_trajectory_pub.publish(joint_trajectory);

        // This will adjust as needed per iteration
        ros::WallDuration(1.0).sleep();
    }

    ros::shutdown();
    
    return 0;
}

