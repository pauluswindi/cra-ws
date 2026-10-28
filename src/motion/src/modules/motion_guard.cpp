#include "modules/motion_guard.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion {

MotionGuard::MotionGuard(
    const RobotKinematics& kinematics,
    double max_joint_velocity
)
: lower_limits_(kinematics.lowerLimits()),
  upper_limits_(kinematics.upperLimits()),
  max_joint_velocity_(max_joint_velocity) {

    if (
        !std::isfinite(max_joint_velocity_) ||
        max_joint_velocity_ <= 0.0
    ) {
        throw std::invalid_argument(
            "Maximum joint velocity must be positive"
        );
    }

    if (
        !lower_limits_.allFinite() ||
        !upper_limits_.allFinite() ||
        (lower_limits_.array() >= upper_limits_.array()).any()
    ) {
        throw std::invalid_argument(
            "Invalid joint limits"
        );
    }
}

bool MotionGuard::filterVelocity(
    const JointVector& velocity,
    const JointVector& current_joints,
    JointVector& output
) {
    output.setZero();

    if (stopped_) {
        return false;
    }

    if (!velocity.allFinite() || !current_joints.allFinite()) {
        stop();
        return false;
    }

    for (int i = 0; i < velocity.size(); ++i) {
        double command = std::clamp(
            velocity[i],
            -max_joint_velocity_,
            max_joint_velocity_
        );

        if (
            (current_joints[i] <= lower_limits_[i] && command < 0.0) ||
            (current_joints[i] >= upper_limits_[i] && command > 0.0)
        ) {
            command = 0.0;
        }

        output[i] = command;
    }

    return true;
}

void MotionGuard::stop() {
    stopped_ = true;
}

void MotionGuard::release() {
    stopped_ = false;
}

bool MotionGuard::isStopped() const {
    return stopped_;
}

}  // namespace motion