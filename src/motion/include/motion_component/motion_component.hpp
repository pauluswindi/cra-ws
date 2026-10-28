#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common.hpp"
#include "controllers/cartesian_controller.hpp"
#include "controllers/execution_monitor.hpp"
#include "controllers/joint_controller.hpp"
#include "kinematics/robot_kinematics.hpp"
#include "logic/robot_initializer.hpp"
#include "modules/data_module.hpp"
#include "modules/joint_feedback.hpp"
#include "modules/motion_guard.hpp"
#include "modules/servo_module.hpp"

namespace motion {

class MotionComponent : public rclcpp::Node {
public:
    explicit MotionComponent(const rclcpp::NodeOptions& options);
    ~MotionComponent();

    void InitComponent();

private:
    using JointVector = RobotKinematics::JointVector;
    using TimePoint = std::chrono::steady_clock::time_point;

    enum class ControlMode {
        Cartesian,
        JointTrajectory
    };

    void Init();
    void InitTimerCallback();
    void MainLoop();
    void RunInitTest();

    void JointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr message
    );

    void TargetPositionCallback(
        const geometry_msgs::msg::Point::SharedPtr message
    );

    void TargetMarkerCallback(
        const visualization_msgs::msg::Marker::SharedPtr message
    );

    void JointTrajectoryCallback(
        const trajectory_msgs::msg::JointTrajectory::SharedPtr message
    );

    void StopCallback(
        const std_msgs::msg::Bool::SharedPtr message
    );

    void SetCartesianTarget(const Eigen::Vector3d& target);

    void RunCartesianControl(const TimePoint& now);
    void RunJointTrajectoryControl(const TimePoint& now);

    bool IsMotionActive() const;
    void CancelMotion();

    void StopWithError(
        const std::string& status,
        const std::string& reason
    );

    bool FilterJointVelocity(
        const JointVector& velocity,
        JointVector& safe_velocity
    );

    void PublishFilteredVelocity(const JointVector& velocity);
    void PublishJointVelocity(const JointVector& velocity);
    void StopJointMotion();

    void PublishStatus(const std::string& status);
    void PublishFailureReason(const std::string& reason);

    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr main_timer_;

    rclcpp::Subscription<
        sensor_msgs::msg::JointState
    >::SharedPtr joint_state_subscriber_;

    rclcpp::Subscription<
        geometry_msgs::msg::Point
    >::SharedPtr target_position_subscriber_;

    rclcpp::Subscription<
        visualization_msgs::msg::Marker
    >::SharedPtr target_marker_subscriber_;

    rclcpp::Subscription<
        trajectory_msgs::msg::JointTrajectory
    >::SharedPtr joint_trajectory_subscriber_;

    rclcpp::Subscription<
        std_msgs::msg::Bool
    >::SharedPtr stop_subscriber_;

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray
    >::SharedPtr joint_velocity_publisher_;

    rclcpp::Publisher<
        std_msgs::msg::String
    >::SharedPtr status_publisher_;

    rclcpp::Publisher<
        std_msgs::msg::String
    >::SharedPtr failure_reason_publisher_;

    std::unique_ptr<DataModule> data_module_;
    std::unique_ptr<RobotKinematics> robot_kinematics_;

    std::unique_ptr<JointFeedback> joint_feedback_;
    std::unique_ptr<MotionGuard> motion_guard_;

    std::unique_ptr<CartesianController> cartesian_controller_;
    std::unique_ptr<JointController> joint_controller_;
    std::unique_ptr<ExecutionMonitor> execution_monitor_;

    std::unique_ptr<ServoModule> servo_module_;
    std::unique_ptr<RobotInitializer> robot_initializer_;

    std::thread init_thread_;

    MotionState::State state_{MotionState::State::Init};
    ControlMode control_mode_{ControlMode::Cartesian};

    std::string robot_mode_{"simulation"};
    std::string target_source_{"legacy"};
    std::string base_frame_;
    std::string last_status_;
    std::string last_failure_reason_;

    bool initialized_{false};
    bool robot_init_started_{false};
};

}  // namespace motion