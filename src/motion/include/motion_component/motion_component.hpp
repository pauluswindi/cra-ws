#ifndef MOTION_COMPONENT_HPP_
#define MOTION_COMPONENT_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "common.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "kinematics/robot_kinematics.hpp"
#include "trajectory/trajectory_generator.hpp"

#include "modules/data_module.hpp"
#include "modules/servo_module.hpp"
#include "logic/robot_initializer.hpp"

namespace motion {

class MotionComponent : public rclcpp::Node {
public:
    explicit MotionComponent(const rclcpp::NodeOptions& options);
    ~MotionComponent();

    void InitComponent();

private:
    using JointVector = RobotKinematics::JointVector;

    enum class ControlMode {
        Cartesian,
        JointTrajectory
    };

    struct JointWaypoint {
        JointVector position = JointVector::Zero();
        double time = 0.0;
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

    void RunCartesianControl(
        const std::chrono::steady_clock::time_point& now
    );

    void RunJointTrajectoryControl(
        const std::chrono::steady_clock::time_point& now
    );

    bool ValidateJointTrajectory(
        const trajectory_msgs::msg::JointTrajectory& message,
        std::vector<JointWaypoint>& waypoints
    ) const;

    void SetCartesianTarget(const Eigen::Vector3d& target);
    void PublishJointVelocity(const JointVector& velocity);
    void StopJointMotion();
    void PublishStatus(const std::string& status);

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

    bool initialized_{false};
    bool joint_feedback_received_{false};
    bool target_received_{false};
    bool robot_init_started_{false};
    bool stop_latched_{false};
    bool joint_trajectory_active_{false};

    std::string robot_mode_{"simulation"};
    std::string target_source_{"legacy"};
    std::string last_status_;

    ControlMode control_mode_{ControlMode::Cartesian};

    MotionState::State state_ =
        MotionState::State::Init;

    std::unique_ptr<DataModule> data_module_;
    std::unique_ptr<ServoModule> servo_module_;
    std::unique_ptr<RobotInitializer> robot_initializer_;
    std::unique_ptr<RobotKinematics> robot_kinematics_;

    JointVector current_joints_ =
        JointVector::Zero();

    Eigen::Vector3d target_position_ =
        Eigen::Vector3d::Zero();

    std::chrono::steady_clock::time_point
        last_feedback_time_{};

    std::chrono::steady_clock::time_point
        trajectory_start_time_{};

    std::chrono::steady_clock::time_point
        joint_trajectory_start_time_{};

    double jacobian_damping_{0.01};

    double tracking_gain_{2.0};
    double max_cartesian_velocity_{0.08};
    double max_joint_velocity_{0.5};
    double target_tolerance_{0.005};
    double feedback_timeout_{0.2};

    double joint_tracking_gain_{2.0};
    double joint_goal_tolerance_{0.02};
    double joint_start_tolerance_{0.10};
    double joint_max_segment_delta_{0.6};

    TrajectoryGenerator trajectory_generator_;

    bool trajectory_enabled_{true};
    bool trajectory_initialized_{false};
    bool trajectory_active_{false};

    Eigen::Vector3d trajectory_target_ =
        Eigen::Vector3d::Zero();

    double trajectory_speed_{0.03};
    double trajectory_distance_step_{0.01};
    double trajectory_replan_threshold_{0.005};

    std::string trajectory_curve_{"smoother_step"};

    std::vector<JointWaypoint> joint_trajectory_;

    std::thread init_thread_;
};

}  // namespace motion

#endif  // MOTION_COMPONENT_HPP_