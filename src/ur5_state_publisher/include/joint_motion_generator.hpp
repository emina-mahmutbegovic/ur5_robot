#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace ur5 {

class JointMotionGenerator {

public:
    JointMotionGenerator() = default;

    JointMotionGenerator(const JointMotionGenerator &) = delete;
    JointMotionGenerator(JointMotionGenerator &&) = delete;

    JointMotionGenerator &operator=(const JointMotionGenerator &) = delete;
    JointMotionGenerator &operator=(JointMotionGenerator &&) = delete;

    ~JointMotionGenerator() = default;
};

}