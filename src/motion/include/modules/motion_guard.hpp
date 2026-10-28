#pragma once

#include "kinematics/robot_kinematics.hpp"

namespace motion {

class MotionGuard {
public:
    using JointVector = RobotKinematics::JointVector;

    MotionGuard(
        const RobotKinematics& kinematics,
        double max_joint_velocity
    );

    bool filterVelocity(
        const JointVector& velocity,
        const JointVector& current_joints,
        JointVector& output
    );

    void stop();
    void release();

    bool isStopped() const;

private:
    JointVector lower_limits_;
    JointVector upper_limits_;

    double max_joint_velocity_;
    bool stopped_{false};
};

}  // namespace motion