#include <ros/ros.h>
#include <string>
#include <vector>

namespace ur5 {

class JointStatePublisher {
   public:
        JointStatePublisher() {
            spinner_.start();
        };

        JointStatePublisher(const JointStatePublisher &) = delete;
        JointStatePublisher(JointStatePublisher &&) = delete;

        JointStatePublisher &operator=(const JointStatePublisher &) = delete;
        JointStatePublisher &operator=(JointStatePublisher &&) = delete;

        ~JointStatePublisher() = default;

        void update(const std::vector<double> &positions, 
                    const std::vector<double> &velocities,
                    const std::vector<double> &accelerations
        );

    private:

    void init() {
        joint_trajectory_pub_ = n.advertise<trajectory_msgs::JointTrajectory>(kControllerCommandTopic, kQueueSize, true);
    }

    // Define constants
    const int kLoopRate = 50;
    const std::string kServiceName = "ur5_state_publisher";
    const int kQueueSize = 10;

    ros::NodeHandle node_handle_;
    ros::AsyncSpinner spinner_(1);

    ros::Publisher joint_trajectory_pub _;

};

 // TODO: Load constants below from config file
const std::string kControllerCommandTopic = "/ur5/eff_joint_traj_controller/command";
const std::string kControllerStateTopic = "/ur5/eff_joint_traj_controller/state";

const std::string kShoulderPanJoint = "shoulder_pan_joint";
const std::string kShoulderLiftJoint = "shoulder_lift_joint";
const std::string kElbowJoint = "elbow_joint";
const std::string kWrist1Joint = "wrist_1_joint";
const std::string kWrist2Joint = "wrist_2_joint";
const std::string kWrist3Joint = "wrist_3_joint";

}