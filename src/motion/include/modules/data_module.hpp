#pragma once

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "kinematics/robot_kinematics.hpp"
#include "modules/servo_module.hpp"

class DataModule {
public:
    struct CartesianConfig {
        double speed;
        double gain;
        double tolerance;
    };

    struct JointConfig {
        double speed;
        double gain;
        double tolerance;
        double start_tolerance;
        double max_segment_delta;

        // Temporary compatibility with JointController
        double goal_timeout;
    };

    struct MonitoringConfig {
        double goal_timeout;
        double stall_timeout;
        double tracking_error_limit;
        double tracking_error_timeout;
    };

    struct MotionConfig {
        std::string control_mode;
        std::string target_source;
        double feedback_timeout;

        CartesianConfig cartesian;
        JointConfig joint;
        MonitoringConfig monitoring;
    };

    struct TrajectoryConfig {
        bool enabled;
        double distance_step;
        std::string curve;
        double replan_threshold;
    };

    explicit DataModule(const rclcpp::Node::SharedPtr& node);

    std::array<double, servo::kServoCount> getInitialPositions() const;

    std::string getRobotModelPath() const;
    std::string getRobotMode() const;
    std::string getBaseFrame() const;
    std::string getEndEffectorFrame() const;

    motion::RobotKinematics::IKConfig getIKConfig() const;
    double getJacobianDamping() const;

    MotionConfig getMotionConfig() const;
    TrajectoryConfig getTrajectoryConfig() const;

private:
    void validateConfig() const;

    rclcpp::Node::SharedPtr node_;
    YAML::Node config_;

    std::string package_share_path_;
};