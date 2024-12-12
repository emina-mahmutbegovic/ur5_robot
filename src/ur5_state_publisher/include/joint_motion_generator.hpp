#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <string>

namespace ur5 {

class JointMotionGenerator {

public:
    JointMotionGenerator() = default;
    JointMotionGenerator(const std::string &urdf_file_path);

    JointMotionGenerator(const JointMotionGenerator &) = delete;
    JointMotionGenerator(JointMotionGenerator &&) = delete;

    JointMotionGenerator &operator=(const JointMotionGenerator &) = delete;
    JointMotionGenerator &operator=(JointMotionGenerator &&) = delete;

    ~JointMotionGenerator();

    int init();

private:
    // Define constants
    const std::string kBaseLink = "base_link";
    const std::string kWrist3Link = "wrist_3_link";

    std::string urdf_file_path_;

    KDL::Tree ur5_tree_;
    KDL::Chain ur5_chain_;
    KDL::ChainFkSolverPos_recursive fk_solver_;
	KDL::ChainIkSolverVel_pinv vel_ik_solver_;
	KDL::ChainIkSolverPos_NR ik_solver_;
};

} // namespace ur5