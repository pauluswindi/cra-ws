#pragma once

#include <chrono>
#include <string>

#include <Eigen/Core>

#include "kinematics/robot_kinematics.hpp"
#include "trajectory/trajectory_generator.hpp"

namespace motion {

class CartesianController {
public:
    using JointVector = RobotKinematics::JointVector;
    using Position = Eigen::Vector3d;
    using TimePoint = std::chrono::steady_clock::time_point;

    struct Config {
        double gain{2.0};
        double max_velocity{0.1};
        double goal_tolerance{0.01};
        double replan_threshold{0.005};
        double jacobian_damping{0.01};
        bool trajectory_enabled{true};
    };

    enum class State {
        Idle,
        Running,
        Completed,
        Error
    };

    struct Result {
        JointVector velocity = JointVector::Zero();
        State state{State::Idle};
        double position_error{0.0};
    };

    CartesianController(
        RobotKinematics& kinematics,
        const Config& config,
        const TrajectoryGenerator::Config& trajectory_config
    );

    bool setTarget(const Position& target);

    Result update(
        const JointVector& current_joints,
        const TimePoint& now
    );

    void cancel();

    bool isActive() const;
    const std::string& lastError() const;

private:
    RobotKinematics& kinematics_;
    Config config_;
    TrajectoryGenerator trajectory_generator_;

    Position target_ = Position::Zero();
    Position trajectory_target_ = Position::Zero();

    TimePoint start_time_{};

    bool target_received_{false};
    bool trajectory_active_{false};
    bool trajectory_initialized_{false};

    std::string last_error_;
};

}  // namespace motion