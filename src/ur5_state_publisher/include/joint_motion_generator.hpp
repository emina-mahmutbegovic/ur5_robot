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