#include "controllers/execution_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion {

ExecutionMonitor::ExecutionMonitor(const Config& config)
: config_(config) {
    const std::array<double, 4> parameters = {
        config_.goal_timeout,
        config_.stall_timeout,
        config_.tracking_error_limit,
        config_.tracking_error_timeout
    };

    for (double value : parameters) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(
                "ExecutionMonitor parameters must be positive and finite"
            );
        }
    }
}

void ExecutionMonitor::reset(const TimePoint& now) {
    start_time_ = now;
    last_update_time_ = now;
    stall_start_time_ = now;

    stall_start_position_.setZero();
    commanded_motion_.fill(0.0);

    tracking_error_active_ = false;
    initialized_ = false;
}

ExecutionMonitor::State ExecutionMonitor::update(
    const TimePoint& now,
    double trajectory_duration,
    const JointVector& reference_position,
    const JointVector& current_position,
    const JointVector& applied_velocity,
    double goal_tolerance
) {
    if (
        !std::isfinite(trajectory_duration) ||
        trajectory_duration <= 0.0 ||
        !std::isfinite(goal_tolerance) ||
        goal_tolerance <= 0.0 ||
        !reference_position.allFinite() ||
        !current_position.allFinite() ||
        !applied_velocity.allFinite() ||
        now < start_time_ ||
        now < last_update_time_
    ) {
        return State::InvalidInput;
    }

    if (!initialized_) {
        stall_start_position_ = current_position;
        initialized_ = true;
    }

    const double elapsed =
        std::chrono::duration<double>(
            now - start_time_
        ).count();

    const double dt =
        std::chrono::duration<double>(
            now - last_update_time_
        ).count();

    const JointVector error =
        reference_position - current_position;

    const double max_error =
        error.cwiseAbs().maxCoeff();

    if (max_error > config_.tracking_error_limit) {
        if (!tracking_error_active_) {
            tracking_error_start_ = now;
            tracking_error_active_ = true;
        }

        const double error_duration =
            std::chrono::duration<double>(
                now - tracking_error_start_
            ).count();

        if (error_duration >= config_.tracking_error_timeout) {
            return State::TrackingError;
        }
    } else {
        tracking_error_active_ = false;
    }

    for (int i = 0; i < 5; ++i) {
        commanded_motion_[i] +=
            std::abs(applied_velocity[i]) * dt;
    }

    last_update_time_ = now;

    const double stall_duration =
        std::chrono::duration<double>(
            now - stall_start_time_
        ).count();

    if (stall_duration >= config_.stall_timeout) {
        for (int i = 0; i < 5; ++i) {
            const double actual_progress =
                std::abs(
                    current_position[i] -
                    stall_start_position_[i]
                );

            const double joint_error =
                std::abs(error[i]);

            const bool movement_requested =
                commanded_motion_[i] >=
                config_.tracking_error_limit;

            const bool no_progress =
                actual_progress < goal_tolerance;

            const bool goal_not_reached =
                joint_error > goal_tolerance;

            if (
                movement_requested &&
                no_progress &&
                goal_not_reached
            ) {
                return State::Stalled;
            }
        }

        stall_start_time_ = now;
        stall_start_position_ = current_position;
        commanded_motion_.fill(0.0);
    }

    if (
        elapsed >=
        trajectory_duration + config_.goal_timeout
    ) {
        return State::GoalTimeout;
    }

    return State::Normal;
}

const char* ExecutionMonitor::stateName(State state) {
    switch (state) {
        case State::Normal:
            return "NORMAL";

        case State::GoalTimeout:
            return "GOAL_TIMEOUT";

        case State::Stalled:
            return "STALLED";

        case State::TrackingError:
            return "TRACKING_ERROR";

        case State::InvalidInput:
            return "INVALID_INPUT";
    }

    return "UNKNOWN";
}

}  // namespace motion