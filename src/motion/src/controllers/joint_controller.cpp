#include "controllers/joint_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace motion {

JointController::JointController(
    const RobotKinematics& kinematics,
    const Config& config,
    const TrajectoryGenerator::Config& trajectory_config
)
: config_(config),
  lower_limits_(kinematics.lowerLimits()),
  upper_limits_(kinematics.upperLimits()),
  trajectory_generator_(trajectory_config) {

    const std::array<double, 5> parameters = {
        config_.gain,
        config_.max_velocity,
        config_.goal_tolerance,
        config_.start_tolerance,
        config_.max_segment_delta
    };

    for (double value : parameters) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(
                "JointController parameters must be positive and finite"
            );
        }
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

bool JointController::validateTrajectory(
    const trajectory_msgs::msg::JointTrajectory& message,
    TrajectoryGenerator::JointPath& path
) {
    constexpr std::array<const char*, 5> joint_names = {
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5"
    };

    if (
        message.joint_names.size() != joint_names.size() ||
        message.points.size() < 2
    ) {
        last_error_ = "Invalid joint names or waypoint count";
        return false;
    }

    std::array<std::size_t, 5> indices{};

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        const auto it = std::find(
            message.joint_names.begin(),
            message.joint_names.end(),
            joint_names[i]
        );

        if (it == message.joint_names.end()) {
            last_error_ = "Missing joint name";
            return false;
        }

        indices[i] = static_cast<std::size_t>(
            std::distance(message.joint_names.begin(), it)
        );
    }

    path.clear();
    path.reserve(message.points.size());

    for (const auto& point : message.points) {
        if (point.positions.size() != joint_names.size()) {
            last_error_ = "Invalid waypoint size";
            return false;
        }

        JointVector position;

        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            position[static_cast<int>(i)] =
                point.positions[indices[i]];
        }

        if (!position.allFinite()) {
            last_error_ = "Invalid joint position";
            return false;
        }

        if (
            (position.array() < lower_limits_.array()).any() ||
            (position.array() > upper_limits_.array()).any()
        ) {
            last_error_ = "Waypoint exceeds joint limits";
            return false;
        }

        if (!path.empty()) {
            const double max_delta =
                (position - path.back())
                    .cwiseAbs()
                    .maxCoeff();

            if (max_delta > config_.max_segment_delta + 1e-9) {
                last_error_ = "Joint segment exceeds maximum delta";
                return false;
            }
        }

        path.push_back(position);
    }

    return true;
}

bool JointController::setTrajectory(
    const trajectory_msgs::msg::JointTrajectory& message,
    const JointVector& current_joints,
    const TimePoint& now
) {
    last_error_.clear();

    if (!current_joints.allFinite()) {
        last_error_ = "Invalid current joint positions";
        return false;
    }

    TrajectoryGenerator::JointPath path;

    if (!validateTrajectory(message, path)) {
        return false;
    }

    const double start_error =
        (path.front() - current_joints)
            .cwiseAbs()
            .maxCoeff();

    if (start_error > config_.start_tolerance) {
        last_error_ = "Trajectory start position mismatch";
        return false;
    }

    try {
        trajectory_generator_.generateJoint(
            path,
            config_.max_velocity
        );
    } catch (const std::exception& error) {
        last_error_ = error.what();
        return false;
    }

    start_time_ = now;
    active_ = true;

    return true;
}

JointController::Result JointController::update(
    const JointVector& current_joints,
    const TimePoint& now
) {
    Result result;

    if (!active_) {
        return result;
    }

    if (!current_joints.allFinite()) {
        last_error_ = "Invalid joint feedback";
        cancel();

        result.state = State::Error;
        return result;
    }

    const double elapsed =
        std::chrono::duration<double>(
            now - start_time_
        ).count();

    if (!std::isfinite(elapsed) || elapsed < 0.0) {
        last_error_ = "Invalid trajectory time";
        cancel();

        result.state = State::Error;
        return result;
    }

    TrajectoryGenerator::JointSample reference;

    try {
        reference = trajectory_generator_.sampleJoint(elapsed);
    } catch (const std::exception& error) {
        last_error_ = error.what();
        cancel();

        result.state = State::Error;
        return result;
    }

    result.reference_position = reference.position;
    result.reference_velocity = reference.velocity;

    if (
        !result.reference_position.allFinite() ||
        !result.reference_velocity.allFinite()
    ) {
        last_error_ = "Invalid trajectory reference";
        cancel();

        result.state = State::Error;
        return result;
    }

    const JointVector error =
        reference.position - current_joints;

    result.velocity =
        reference.velocity + config_.gain * error;

    if (!result.velocity.allFinite()) {
        last_error_ = "Invalid joint velocity command";
        cancel();

        result.velocity.setZero();
        result.state = State::Error;
        return result;
    }

    if (elapsed >= trajectory_generator_.jointTotalTime()) {
        const double final_error =
            error.cwiseAbs().maxCoeff();

        if (final_error <= config_.goal_tolerance) {
            cancel();

            result.velocity.setZero();
            result.state = State::Completed;
            return result;
        }
    }

    result.state = State::Running;
    return result;
}

void JointController::cancel() {
    active_ = false;
}

bool JointController::isActive() const {
    return active_;
}

double JointController::totalTime() const {
    return trajectory_generator_.jointTotalTime();
}

double JointController::goalTolerance() const {
    return config_.goal_tolerance;
}

const std::string& JointController::lastError() const {
    return last_error_;
}

}  // namespace motion