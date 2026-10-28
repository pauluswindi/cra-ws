#pragma once

#include <chrono>

#include <sensor_msgs/msg/joint_state.hpp>

#include "kinematics/robot_kinematics.hpp"

namespace motion {

class JointFeedback {
public:
    using JointVector = RobotKinematics::JointVector;
    using TimePoint = std::chrono::steady_clock::time_point;

    JointFeedback(
        const RobotKinematics& kinematics,
        double timeout
    );

    bool update(
        const sensor_msgs::msg::JointState& message,
        TimePoint now = std::chrono::steady_clock::now()
    );

    bool isValid() const;
    bool isFresh(
        TimePoint now = std::chrono::steady_clock::now()
    ) const;

    const JointVector& positions() const;

private:
    JointVector lower_limits_;
    JointVector upper_limits_;
    JointVector positions_ = JointVector::Zero();

    TimePoint last_update_{};

    double timeout_;
    bool valid_{false};
};

}  // namespace motion