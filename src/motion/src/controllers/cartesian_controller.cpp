#include "controllers/cartesian_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <stdexcept>

#include "kinematics/jacobian.hpp"

namespace motion {

CartesianController::CartesianController(
    RobotKinematics& kinematics,
    const Config& config,
    const TrajectoryGenerator::Config& trajectory_config
)
: kinematics_(kinematics),
  config_(config),
  trajectory_generator_(trajectory_config) {

    const std::array<double, 5> parameters = {
        config_.gain,
        config_.max_velocity,
        config_.goal_tolerance,
        config_.replan_threshold,
        config_.jacobian_damping
    };

    for (double value : parameters) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(
                "CartesianController parameters must be positive and finite"
            );
        }
    }
}

bool CartesianController::setTarget(const Position& target) {
    if (!target.allFinite()) {
        last_error_ = "Invalid Cartesian target";
        return false;
    }

    // Ignore changes smaller than the replanning threshold.
    if (
        target_received_ &&
        (target - target_).norm() <= config_.replan_threshold
    ) {
        return true;
    }

    target_ = target;
    target_received_ = true;
    trajectory_initialized_ = false;
    trajectory_active_ = false;

    last_error_.clear();
    return true;
}

CartesianController::Result CartesianController::update(
    const JointVector& current_joints,
    const TimePoint& now
) {
    Result result;

    if (!target_received_) {
        return result;
    }

    if (!current_joints.allFinite()) {
        last_error_ = "Invalid joint feedback";
        cancel();
        result.state = State::Error;
        return result;
    }

    try {
        kinematics_.setJointPositions(current_joints);

        const Position current_position =
            kinematics_.forwardKinematics(
                RobotKinematics::Frame::Base
            );

        if (!current_position.allFinite()) {
            throw std::runtime_error(
                "Invalid forward kinematics result"
            );
        }

        result.position_error =
            (target_ - current_position).norm();

        if (!std::isfinite(result.position_error)) {
            throw std::runtime_error(
                "Invalid Cartesian position error"
            );
        }

        if (result.position_error <= config_.goal_tolerance) {
            cancel();
            result.state = State::Completed;
            return result;
        }

        Position desired_position = target_;
        Position desired_velocity = Position::Zero();

        if (config_.trajectory_enabled) {
            if (!trajectory_initialized_) {
                trajectory_generator_.generate({
                    current_position,
                    target_
                });

                trajectory_target_ = target_;
                start_time_ = now;

                trajectory_initialized_ = true;
                trajectory_active_ = true;
            }

            if (trajectory_active_) {
                const double elapsed =
                    std::chrono::duration<double>(
                        now - start_time_
                    ).count();

                const auto reference =
                    trajectory_generator_.sample(elapsed);

                desired_position = reference.position;
                desired_velocity = reference.velocity;
            }
        }

        const Position position_error =
            desired_position - current_position;

        Position cartesian_velocity =
            desired_velocity + config_.gain * position_error;

        if (!cartesian_velocity.allFinite()) {
            throw std::runtime_error(
                "Invalid Cartesian velocity"
            );
        }

        const double speed = cartesian_velocity.norm();

        if (speed > config_.max_velocity) {
            cartesian_velocity *= config_.max_velocity / speed;
        }

        const auto jacobian = Jacobian::computeJacobian(
            kinematics_,
            RobotKinematics::Frame::Base
        );

        result.velocity = Jacobian::inverseJacobian(
            jacobian,
            cartesian_velocity,
            config_.jacobian_damping
        );

        if (!result.velocity.allFinite()) {
            throw std::runtime_error(
                "Invalid joint velocity from Jacobian"
            );
        }

        result.state = State::Running;

    } catch (const std::exception& error) {
        last_error_ = error.what();
        cancel();

        result.velocity.setZero();
        result.state = State::Error;
    }

    return result;
}

void CartesianController::cancel() {
    target_received_ = false;
    trajectory_active_ = false;
    trajectory_initialized_ = false;
}

bool CartesianController::isActive() const {
    return target_received_;
}

const std::string& CartesianController::lastError() const {
    return last_error_;
}

}  // namespace motion