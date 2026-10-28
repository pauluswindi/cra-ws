#pragma once

#include <array>
#include <chrono>
#include <string>

#include "kinematics/robot_kinematics.hpp"

namespace motion {

class ExecutionMonitor {
public:
    using JointVector = RobotKinematics::JointVector;
    using TimePoint = std::chrono::steady_clock::time_point;

    struct Config {
        double goal_timeout;
        double stall_timeout;
        double tracking_error_limit;
        double tracking_error_timeout;
    };

    enum class State {
        Normal,
        GoalTimeout,
        Stalled,
        TrackingError,
        InvalidInput
    };

    explicit ExecutionMonitor(const Config& config);

    void reset(const TimePoint& now);

    State update(
        const TimePoint& now,
        double trajectory_duration,
        const JointVector& reference_position,
        const JointVector& current_position,
        const JointVector& applied_velocity,
        double goal_tolerance
    );

    static const char* stateName(State state);

private:
    Config config_;

    TimePoint start_time_{};
    TimePoint last_update_time_{};
    TimePoint stall_start_time_{};
    TimePoint tracking_error_start_{};

    JointVector stall_start_position_ = JointVector::Zero();

    std::array<double, 5> commanded_motion_{};

    bool initialized_{false};
    bool tracking_error_active_{false};
};

}  // namespace motion