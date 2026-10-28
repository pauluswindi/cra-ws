#pragma once

#include <chrono>
#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "kinematics/robot_kinematics.hpp"
#include "trajectory/trajectory_generator.hpp"

namespace motion {

class JointController {
public:
    using JointVector = RobotKinematics::JointVector;
    using TimePoint = std::chrono::steady_clock::time_point;

    struct Config {
        double gain{2.0};
        double max_velocity{0.5};
        double goal_tolerance{0.02};
        double start_tolerance{0.10};
        double max_segment_delta{0.6};
    };

    enum class State {
        Idle,
        Running,
        Completed,
        Error
    };

    struct Result {
        JointVector velocity = JointVector::Zero();
        JointVector reference_position = JointVector::Zero();
        JointVector reference_velocity = JointVector::Zero();

        State state{State::Idle};
    };

    JointController(
        const RobotKinematics& kinematics,
        const Config& config,
        const TrajectoryGenerator::Config& trajectory_config
    );

    bool setTrajectory(
        const trajectory_msgs::msg::JointTrajectory& message,
        const JointVector& current_joints,
        const TimePoint& now
    );

    Result update(
        const JointVector& current_joints,
        const TimePoint& now
    );

    void cancel();

    bool isActive() const;
    double totalTime() const;
    double goalTolerance() const;
    const std::string& lastError() const;

private:
    bool validateTrajectory(
        const trajectory_msgs::msg::JointTrajectory& message,
        TrajectoryGenerator::JointPath& path
    );

    Config config_;

    JointVector lower_limits_;
    JointVector upper_limits_;

    TrajectoryGenerator trajectory_generator_;

    TimePoint start_time_{};
    bool active_{false};

    std::string last_error_;
};

}  // namespace motion