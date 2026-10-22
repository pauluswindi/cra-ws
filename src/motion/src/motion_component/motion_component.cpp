#include "motion_component/motion_component.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "kinematics/jacobian.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace motion {

MotionComponent::MotionComponent(
    const rclcpp::NodeOptions& options
)
: Node("motion_component", options) {
    Init();
    RCLCPP_INFO(get_logger(), "MotionComponent constructed");
}

MotionComponent::~MotionComponent() {
    if (main_timer_) {
        main_timer_->cancel();
    }

    if (init_timer_) {
        init_timer_->cancel();
    }

    if (init_thread_.joinable()) {
        init_thread_.join();
    }
}

void MotionComponent::Init() {
    init_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MotionComponent::InitTimerCallback, this)
    );
}

void MotionComponent::InitTimerCallback() {
    if (initialized_) {
        return;
    }

    init_timer_->cancel();

    try {
        InitComponent();
        initialized_ = true;

        RCLCPP_INFO(
            get_logger(),
            "MotionComponent initialized"
        );
    } catch (const std::exception& error) {
        RCLCPP_ERROR(
            get_logger(),
            "Initialization failed: %s",
            error.what()
        );
    }
}

void MotionComponent::InitComponent() {
    auto self = shared_from_this();

    data_module_ = std::make_unique<DataModule>(self);
    robot_mode_ = data_module_->getRobotMode();

    if (robot_mode_ != "simulation" && robot_mode_ != "robot") {
        throw std::runtime_error(
            "Invalid robot mode: " + robot_mode_
        );
    }

    const std::string xml_path =
        data_module_->getRobotModelPath();

    robot_kinematics_ = std::make_unique<RobotKinematics>(
        xml_path,
        data_module_->getBaseFrame(),
        data_module_->getEndEffectorFrame()
    );

    robot_kinematics_->setIKConfig(
        data_module_->getIKConfig()
    );

    jacobian_damping_ =
        data_module_->getJacobianDamping();

    if (
        !std::isfinite(jacobian_damping_) ||
        jacobian_damping_ <= 0.0
    ) {
        throw std::runtime_error(
            "Invalid Jacobian damping"
        );
    }

    joint_state_subscriber_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(
                &MotionComponent::JointStateCallback,
                this,
                std::placeholders::_1
            )
        );

    if (robot_mode_ == "simulation") {
        tracking_gain_ = declare_parameter<double>(
            "tracking_gain", 2.0
        );

        max_cartesian_velocity_ = declare_parameter<double>(
            "max_cartesian_velocity", 0.08
        );

        max_joint_velocity_ = declare_parameter<double>(
            "max_joint_velocity", 0.5
        );

        target_tolerance_ = declare_parameter<double>(
            "target_tolerance", 0.005
        );

        feedback_timeout_ = declare_parameter<double>(
            "feedback_timeout", 0.2
        );

        trajectory_enabled_ = declare_parameter<bool>(
            "trajectory.enabled", true
        );

        trajectory_speed_ = declare_parameter<double>(
            "trajectory.speed", 0.03
        );

        trajectory_distance_step_ = declare_parameter<double>(
            "trajectory.distance_step", 0.01
        );

        trajectory_curve_ = declare_parameter<std::string>(
            "trajectory.curve", "smoother_step"
        );

        trajectory_replan_threshold_ =
            declare_parameter<double>(
                "trajectory.replan_threshold", 0.005
            );

        joint_tracking_gain_ = declare_parameter<double>(
            "joint_tracking_gain", 2.0
        );

        joint_goal_tolerance_ = declare_parameter<double>(
            "joint_goal_tolerance", 0.02
        );

        joint_start_tolerance_ = declare_parameter<double>(
            "joint_start_tolerance", 0.10
        );

        joint_max_segment_delta_ = declare_parameter<double>(
            "joint_max_segment_delta", 0.6
        );

        const std::string mode = declare_parameter<std::string>(
            "control_mode", "cartesian"
        );

        target_source_ = declare_parameter<std::string>(
            "target_source", "legacy"
        );

        if (mode == "cartesian") {
            control_mode_ = ControlMode::Cartesian;
        } else if (mode == "joint_trajectory") {
            control_mode_ = ControlMode::JointTrajectory;
        } else {
            throw std::runtime_error(
                "Invalid control_mode: " + mode
            );
        }

        if (
            target_source_ != "legacy" &&
            target_source_ != "simulation_marker"
        ) {
            throw std::runtime_error(
                "Invalid target_source: " + target_source_
            );
        }

        const std::array<double, 12> parameters = {
            tracking_gain_,
            max_cartesian_velocity_,
            max_joint_velocity_,
            target_tolerance_,
            feedback_timeout_,
            trajectory_speed_,
            trajectory_distance_step_,
            trajectory_replan_threshold_,
            joint_tracking_gain_,
            joint_goal_tolerance_,
            joint_start_tolerance_,
            joint_max_segment_delta_
        };

        for (double value : parameters) {
            if (!std::isfinite(value) || value <= 0.0) {
                throw std::runtime_error(
                    "Motion parameters must be positive and finite"
                );
            }
        }

        TrajectoryGenerator::Config trajectory_config;
        trajectory_config.speed = trajectory_speed_;
        trajectory_config.distance_step =
            trajectory_distance_step_;
        trajectory_config.curve = trajectory_curve_;

        trajectory_generator_.setConfig(trajectory_config);

        if (target_source_ == "legacy") {
            target_position_subscriber_ =
                create_subscription<geometry_msgs::msg::Point>(
                    "/motion/target_position",
                    10,
                    std::bind(
                        &MotionComponent::TargetPositionCallback,
                        this,
                        std::placeholders::_1
                    )
                );
        } else {
            rclcpp::QoS target_qos(1);
            target_qos.reliable();
            target_qos.transient_local();

            target_marker_subscriber_ =
                create_subscription<
                    visualization_msgs::msg::Marker
                >(
                    "/simulation/target_marker",
                    target_qos,
                    std::bind(
                        &MotionComponent::TargetMarkerCallback,
                        this,
                        std::placeholders::_1
                    )
                );
        }

        joint_trajectory_subscriber_ =
            create_subscription<
                trajectory_msgs::msg::JointTrajectory
            >(
                "/motion/joint_path",
                10,
                std::bind(
                    &MotionComponent::JointTrajectoryCallback,
                    this,
                    std::placeholders::_1
                )
            );

        stop_subscriber_ =
            create_subscription<std_msgs::msg::Bool>(
                "/motion/stop_requested",
                10,
                std::bind(
                    &MotionComponent::StopCallback,
                    this,
                    std::placeholders::_1
                )
            );

        joint_velocity_publisher_ =
            create_publisher<
                std_msgs::msg::Float64MultiArray
            >(
                "/motion/joint_velocity_command",
                10
            );

        rclcpp::QoS status_qos(1);
        status_qos.reliable();
        status_qos.transient_local();

        status_publisher_ =
            create_publisher<std_msgs::msg::String>(
                "/motion/execution_status",
                status_qos
            );

        RCLCPP_INFO(
            get_logger(),
            "Control mode: %s | target source: %s",
            mode.c_str(),
            target_source_.c_str()
        );

        PublishStatus("IDLE");
    }

    if (robot_mode_ == "robot") {
        servo_module_ = std::make_unique<ServoModule>(self);

        robot_initializer_ =
            std::make_unique<RobotInitializer>(
                servo_module_.get(),
                data_module_.get()
            );
    }

    main_timer_ = create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&MotionComponent::MainLoop, this)
    );

    RCLCPP_INFO(
        get_logger(),
        "Waiting for joint feedback"
    );
}

void MotionComponent::JointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr message
) {
    if (!robot_kinematics_) {
        return;
    }

    constexpr std::array<const char*, 5> joint_names = {
        "joint1", "joint2", "joint3", "joint4", "joint5"
    };

    if (message->name.size() != message->position.size()) {
        joint_feedback_received_ = false;
        return;
    }

    JointVector joints;

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        const auto it = std::find(
            message->name.begin(),
            message->name.end(),
            joint_names[i]
        );

        if (it == message->name.end()) {
            joint_feedback_received_ = false;
            return;
        }

        const auto index = std::distance(
            message->name.begin(),
            it
        );

        joints[static_cast<int>(i)] =
            message->position[index];
    }

    if (!joints.allFinite()) {
        joint_feedback_received_ = false;
        return;
    }

    const auto lower = robot_kinematics_->lowerLimits();
    const auto upper = robot_kinematics_->upperLimits();

    constexpr double tolerance = 1e-3;

    if (
        (joints.array() < lower.array() - tolerance).any() ||
        (joints.array() > upper.array() + tolerance).any()
    ) {
        joint_feedback_received_ = false;
        return;
    }

    current_joints_ = joints;
    joint_feedback_received_ = true;

    last_feedback_time_ =
        std::chrono::steady_clock::now();
}

void MotionComponent::SetCartesianTarget(
    const Eigen::Vector3d& target
) {
    if (
        control_mode_ != ControlMode::Cartesian ||
        stop_latched_
    ) {
        return;
    }

    if (!target.allFinite()) {
        target_received_ = false;
        StopJointMotion();
        return;
    }

    const bool changed =
        !target_received_ ||
        (target - target_position_).norm() >
            trajectory_replan_threshold_;

    target_position_ = target;
    target_received_ = true;

    if (changed) {
        RCLCPP_INFO(
            get_logger(),
            "Cartesian target: [%.3f, %.3f, %.3f]",
            target.x(),
            target.y(),
            target.z()
        );
    }
}

void MotionComponent::TargetPositionCallback(
    const geometry_msgs::msg::Point::SharedPtr message
) {
    SetCartesianTarget(
        Eigen::Vector3d(
            message->x,
            message->y,
            message->z
        )
    );
}

void MotionComponent::TargetMarkerCallback(
    const visualization_msgs::msg::Marker::SharedPtr message
) {
    if (
        message->header.frame_id !=
        data_module_->getBaseFrame()
    ) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Target frame mismatch: %s",
            message->header.frame_id.c_str()
        );
        return;
    }

    if (
        message->action !=
        visualization_msgs::msg::Marker::ADD
    ) {
        return;
    }

    SetCartesianTarget(
        Eigen::Vector3d(
            message->pose.position.x,
            message->pose.position.y,
            message->pose.position.z
        )
    );
}

bool MotionComponent::ValidateJointTrajectory(
    const trajectory_msgs::msg::JointTrajectory& message,
    std::vector<JointWaypoint>& waypoints
) const {
    if (!robot_kinematics_) {
        return false;
    }

    constexpr std::array<const char*, 5> expected_names = {
        "joint1", "joint2", "joint3", "joint4", "joint5"
    };

    if (
        message.joint_names.size() != 5 ||
        message.points.size() < 2
    ) {
        return false;
    }

    std::array<std::size_t, 5> joint_indices{};

    for (std::size_t i = 0; i < 5; ++i) {
        const auto it = std::find(
            message.joint_names.begin(),
            message.joint_names.end(),
            expected_names[i]
        );

        if (it == message.joint_names.end()) {
            return false;
        }

        joint_indices[i] = static_cast<std::size_t>(
            std::distance(
                message.joint_names.begin(),
                it
            )
        );
    }

    const auto lower = robot_kinematics_->lowerLimits();
    const auto upper = robot_kinematics_->upperLimits();

    waypoints.clear();
    waypoints.reserve(message.points.size());

    for (const auto& point : message.points) {
        if (point.positions.size() != 5) {
            return false;
        }

        JointWaypoint waypoint;

        waypoint.time =
            static_cast<double>(point.time_from_start.sec) +
            static_cast<double>(
                point.time_from_start.nanosec
            ) * 1e-9;

        for (std::size_t i = 0; i < 5; ++i) {
            waypoint.position[static_cast<int>(i)] =
                point.positions[joint_indices[i]];
        }

        if (
            !waypoint.position.allFinite() ||
            !std::isfinite(waypoint.time) ||
            waypoint.time < 0.0
        ) {
            return false;
        }

        if (
            (waypoint.position.array() < lower.array()).any() ||
            (waypoint.position.array() > upper.array()).any()
        ) {
            return false;
        }

        if (!waypoints.empty()) {
            const auto& previous = waypoints.back();

            const double dt =
                waypoint.time - previous.time;

            if (dt <= 0.0) {
                return false;
            }

            const JointVector delta =
                waypoint.position - previous.position;

            if (
                delta.cwiseAbs().maxCoeff() >
                joint_max_segment_delta_ + 1e-9
            ) {
                return false;
            }

            const double peak_velocity =
                1.5 * delta.cwiseAbs().maxCoeff() / dt;

            if (peak_velocity > max_joint_velocity_) {
                return false;
            }
        }

        waypoints.push_back(waypoint);
    }

    return std::abs(waypoints.front().time) <= 1e-9;
}

void MotionComponent::JointTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr message
) {
    if (
        control_mode_ != ControlMode::JointTrajectory ||
        stop_latched_
    ) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    const double feedback_age =
        std::chrono::duration<double>(
            now - last_feedback_time_
        ).count();

    if (
        !joint_feedback_received_ ||
        feedback_age > feedback_timeout_
    ) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory rejected: feedback unavailable"
        );
        return;
    }

    std::vector<JointWaypoint> waypoints;

    if (!ValidateJointTrajectory(*message, waypoints)) {
        RCLCPP_WARN(
            get_logger(),
            "Rejected invalid joint trajectory"
        );
        return;
    }

    const double start_error =
        (
            waypoints.front().position - current_joints_
        ).cwiseAbs().maxCoeff();

    if (start_error > joint_start_tolerance_) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory start mismatch: %.4f rad",
            start_error
        );
        return;
    }

    joint_trajectory_ = std::move(waypoints);
    joint_trajectory_start_time_ = now;
    joint_trajectory_active_ = true;

    PublishStatus("EXECUTING");

    RCLCPP_INFO(
        get_logger(),
        "Joint trajectory accepted: %zu waypoints",
        joint_trajectory_.size()
    );
}

void MotionComponent::StopCallback(
    const std_msgs::msg::Bool::SharedPtr message
) {
    if (message->data) {
        stop_latched_ = true;
        target_received_ = false;

        joint_trajectory_active_ = false;
        joint_trajectory_.clear();

        trajectory_active_ = false;
        trajectory_initialized_ = false;

        StopJointMotion();
        PublishStatus("STOPPED");

        RCLCPP_WARN(
            get_logger(),
            "Motion stopped and trajectory cancelled"
        );

        return;
    }

    if (!stop_latched_) {
        return;
    }

    stop_latched_ = false;

    StopJointMotion();
    PublishStatus("IDLE");

    RCLCPP_INFO(
        get_logger(),
        "Stop released; waiting for a new command"
    );
}

void MotionComponent::PublishStatus(
    const std::string& status
) {
    if (status == last_status_) {
        return;
    }

    last_status_ = status;

    if (!status_publisher_) {
        return;
    }

    std_msgs::msg::String message;
    message.data = status;

    status_publisher_->publish(message);
}

void MotionComponent::StopJointMotion() {
    if (!joint_velocity_publisher_) {
        return;
    }

    std_msgs::msg::Float64MultiArray message;
    message.data.assign(5, 0.0);

    joint_velocity_publisher_->publish(message);
}

void MotionComponent::PublishJointVelocity(
    const JointVector& velocity
) {
    if (!joint_velocity_publisher_) {
        return;
    }

    if (!velocity.allFinite()) {
        StopJointMotion();
        PublishStatus("CONTROL_ERROR");
        return;
    }

    JointVector command = velocity;

    const auto lower = robot_kinematics_->lowerLimits();
    const auto upper = robot_kinematics_->upperLimits();

    for (int i = 0; i < 5; ++i) {
        command[i] = std::clamp(
            command[i],
            -max_joint_velocity_,
            max_joint_velocity_
        );

        if (
            (current_joints_[i] <= lower[i] &&
             command[i] < 0.0) ||
            (current_joints_[i] >= upper[i] &&
             command[i] > 0.0)
        ) {
            command[i] = 0.0;
        }
    }

    std_msgs::msg::Float64MultiArray message;
    message.data.resize(5);

    for (int i = 0; i < 5; ++i) {
        message.data[i] = command[i];
    }

    joint_velocity_publisher_->publish(message);
}

void MotionComponent::RunJointTrajectoryControl(
    const std::chrono::steady_clock::time_point& now
) {
    if (
        !joint_trajectory_active_ ||
        joint_trajectory_.empty()
    ) {
        StopJointMotion();
        if (last_status_ == "WAITING_FOR_FEEDBACK") {
            PublishStatus("IDLE");
        }
        return;
    }

    const double elapsed =
        std::chrono::duration<double>(
            now - joint_trajectory_start_time_
        ).count();

    const auto& final_waypoint =
        joint_trajectory_.back();

    JointVector desired_position =
        final_waypoint.position;

    JointVector desired_velocity =
        JointVector::Zero();

    if (elapsed < final_waypoint.time) {
        const auto it = std::upper_bound(
            joint_trajectory_.begin(),
            joint_trajectory_.end(),
            elapsed,
            [](double value, const JointWaypoint& waypoint) {
                return value < waypoint.time;
            }
        );

        const std::size_t index =
            static_cast<std::size_t>(
                std::distance(
                    joint_trajectory_.begin(),
                    it
                )
            );

        if (
            index == 0 ||
            index >= joint_trajectory_.size()
        ) {
            joint_trajectory_active_ = false;
            joint_trajectory_.clear();
            stop_latched_ = true;

            StopJointMotion();
            PublishStatus("CONTROL_ERROR");
            return;
        }

        const auto& start = joint_trajectory_[index - 1];
        const auto& goal = joint_trajectory_[index];

        const double duration =
            goal.time - start.time;

        const double s = std::clamp(
            (elapsed - start.time) / duration,
            0.0,
            1.0
        );

        const double progress =
            s * s * (3.0 - 2.0 * s);

        const double progress_rate =
            6.0 * s * (1.0 - s) / duration;

        const JointVector delta =
            goal.position - start.position;

        desired_position =
            start.position + progress * delta;

        desired_velocity =
            progress_rate * delta;
    }

    const JointVector error =
        desired_position - current_joints_;

    const JointVector command =
        desired_velocity + joint_tracking_gain_ * error;

    PublishJointVelocity(command);

    if (elapsed >= final_waypoint.time) {
        const double final_error =
            (
                final_waypoint.position - current_joints_
            ).cwiseAbs().maxCoeff();

        if (final_error <= joint_goal_tolerance_) {
            joint_trajectory_active_ = false;
            joint_trajectory_.clear();

            StopJointMotion();
            PublishStatus("COMPLETED");
        } else {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Joint endpoint error: %.4f rad",
                final_error
            );
        }
    }
}

void MotionComponent::RunCartesianControl(
    const std::chrono::steady_clock::time_point& now
) {
    if (!target_received_) {
        StopJointMotion();
        return;
    }

    robot_kinematics_->setJointPositions(
        current_joints_
    );

    const Eigen::Vector3d current_position =
        robot_kinematics_->forwardKinematics(
            RobotKinematics::Frame::Base
        );

    if (!current_position.allFinite()) {
        stop_latched_ = true;
        StopJointMotion();
        PublishStatus("CONTROL_ERROR");
        return;
    }

    const double final_error =
        (target_position_ - current_position).norm();

    if (!std::isfinite(final_error)) {
        stop_latched_ = true;
        StopJointMotion();
        PublishStatus("CONTROL_ERROR");
        return;
    }

    Eigen::Vector3d desired_position =
        target_position_;

    Eigen::Vector3d desired_velocity =
        Eigen::Vector3d::Zero();

    if (trajectory_enabled_) {
        const bool target_changed =
            !trajectory_initialized_ ||
            (target_position_ - trajectory_target_).norm()
                > trajectory_replan_threshold_;

        if (target_changed) {
            trajectory_target_ = target_position_;
            trajectory_initialized_ = true;
            trajectory_active_ = false;

            if (final_error > target_tolerance_) {
                try {
                    trajectory_generator_.generate({
                        current_position,
                        target_position_
                    });

                    trajectory_start_time_ = now;
                    trajectory_active_ = true;

                    RCLCPP_INFO(
                        get_logger(),
                        "Cartesian trajectory: %.3f m, %.3f s",
                        trajectory_generator_.totalDistance(),
                        trajectory_generator_.totalTime()
                    );
                } catch (const std::exception& error) {
                    stop_latched_ = true;
                    StopJointMotion();
                    PublishStatus("CONTROL_ERROR");

                    RCLCPP_ERROR(
                        get_logger(),
                        "Trajectory generation failed: %s",
                        error.what()
                    );
                    return;
                }
            }
        }

        if (trajectory_active_) {
            const double elapsed =
                std::chrono::duration<double>(
                    now - trajectory_start_time_
                ).count();

            const auto reference =
                trajectory_generator_.sample(elapsed);

            desired_position = reference.position;
            desired_velocity = reference.velocity;
        }
    }

    if (final_error <= target_tolerance_) {
        trajectory_active_ = false;
        StopJointMotion();
        PublishStatus("COMPLETED");
        return;
    }

    const Eigen::Vector3d position_error =
        desired_position - current_position;

    Eigen::Vector3d cartesian_velocity =
        desired_velocity + tracking_gain_ * position_error;

    if (!cartesian_velocity.allFinite()) {
        stop_latched_ = true;
        StopJointMotion();
        PublishStatus("CONTROL_ERROR");
        return;
    }

    const double velocity_norm =
        cartesian_velocity.norm();

    if (velocity_norm > max_cartesian_velocity_) {
        cartesian_velocity *=
            max_cartesian_velocity_ / velocity_norm;
    }

    const auto jacobian = Jacobian::computeJacobian(
        *robot_kinematics_,
        RobotKinematics::Frame::Base
    );

    const JointVector joint_velocity =
        Jacobian::inverseJacobian(
            jacobian,
            cartesian_velocity,
            jacobian_damping_
        );

    if (!joint_velocity.allFinite()) {
        stop_latched_ = true;
        StopJointMotion();
        PublishStatus("CONTROL_ERROR");
        return;
    }

    PublishJointVelocity(joint_velocity);
    PublishStatus("EXECUTING");

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Cartesian tracking error: %.3f mm",
        final_error * 1000.0
    );
}

void MotionComponent::MainLoop() {
    if (!initialized_ || !robot_kinematics_) {
        return;
    }

    if (robot_mode_ == "robot") {
        if (state_ == MotionState::State::Init) {
            RunInitTest();
        }
        return;
    }

    if (robot_mode_ != "simulation") {
        return;
    }

    if (stop_latched_) {
        StopJointMotion();
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    if (!joint_feedback_received_) {
        if (joint_trajectory_active_) {
            joint_trajectory_active_ = false;
            joint_trajectory_.clear();
            stop_latched_ = true;
            StopJointMotion();
            PublishStatus("INVALID_FEEDBACK");
            RCLCPP_ERROR(
                get_logger(),
                "Joint feedback invalid; trajectory cancelled"
            );
        } else {
            StopJointMotion();
            PublishStatus("WAITING_FOR_FEEDBACK");
        }
        return;
    }

    const double feedback_age =
        std::chrono::duration<double>(
            now - last_feedback_time_
        ).count();

    if (feedback_age > feedback_timeout_) {
        joint_trajectory_active_ = false;
        joint_trajectory_.clear();

        trajectory_active_ = false;
        trajectory_initialized_ = false;

        target_received_ = false;
        stop_latched_ = true;

        StopJointMotion();
        PublishStatus("FEEDBACK_TIMEOUT");

        RCLCPP_ERROR(
            get_logger(),
            "Joint feedback timed out; motion latched stopped"
        );
        return;
    }

    if (control_mode_ == ControlMode::JointTrajectory) {
        RunJointTrajectoryControl(now);
    } else {
        RunCartesianControl(now);
    }
}

void MotionComponent::RunInitTest() {
    if (!robot_initializer_ || robot_init_started_) {
        return;
    }

    robot_init_started_ = true;

    init_thread_ = std::thread([this]() {
        try {
            robot_initializer_->PosRobotInit();
        } catch (const std::exception& error) {
            RCLCPP_ERROR(
                get_logger(),
                "Robot initialization failed: %s",
                error.what()
            );
        }
    });
}

}  // namespace motion

RCLCPP_COMPONENTS_REGISTER_NODE(motion::MotionComponent)