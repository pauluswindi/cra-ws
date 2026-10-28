#include "modules/joint_feedback.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace motion {

JointFeedback::JointFeedback(
    const RobotKinematics& kinematics,
    double timeout
)
: lower_limits_(kinematics.lowerLimits()),
  upper_limits_(kinematics.upperLimits()),
  timeout_(timeout) {

    if (!std::isfinite(timeout_) || timeout_ <= 0.0) {
        throw std::invalid_argument(
            "Joint feedback timeout must be positive"
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

bool JointFeedback::update(
    const sensor_msgs::msg::JointState& message,
    TimePoint now
) {
    valid_ = false;

    constexpr std::array<const char*, 5> names = {
        "joint1", "joint2", "joint3", "joint4", "joint5"
    };

    if (message.name.size() != message.position.size()) {
        return false;
    }

    JointVector joints;

    for (std::size_t i = 0; i < names.size(); ++i) {
        const auto it = std::find(
            message.name.begin(),
            message.name.end(),
            names[i]
        );

        if (it == message.name.end()) {
            return false;
        }

        const auto index = static_cast<std::size_t>(
            std::distance(message.name.begin(), it)
        );

        joints[static_cast<int>(i)] = message.position[index];
    }

    constexpr double tolerance = 1e-3;

    if (
        !joints.allFinite() ||
        (joints.array() < lower_limits_.array() - tolerance).any() ||
        (joints.array() > upper_limits_.array() + tolerance).any()
    ) {
        return false;
    }

    positions_ = joints;
    last_update_ = now;
    valid_ = true;

    return true;
}

bool JointFeedback::isValid() const {
    return valid_;
}

bool JointFeedback::isFresh(TimePoint now) const {
    if (!valid_ || now < last_update_) {
        return false;
    }

    const double age =
        std::chrono::duration<double>(
            now - last_update_
        ).count();

    return age <= timeout_;
}

const JointFeedback::JointVector&
JointFeedback::positions() const {
    return positions_;
}

}  // namespace motion