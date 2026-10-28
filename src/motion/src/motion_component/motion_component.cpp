#include "motion_component/motion_component.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>

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
    base_frame_ = data_module_->getBaseFrame();

    robot_kinematics_ = std::make_unique<RobotKinematics>(
        data_module_->getRobotModelPath(),
        base_frame_,
        data_module_->getEndEffectorFrame()
    );

    robot_kinematics_->setIKConfig(
        data_module_->getIKConfig()
    );

    if (robot_mode_ == "simulation") {
        const auto motion_config =
            data_module_->getMotionConfig();

        const auto trajectory_config =
            data_module_->getTrajectoryConfig();

        target_source_ = motion_config.target_source;

        if (motion_config.control_mode == "cartesian") {
            control_mode_ = ControlMode::Cartesian;
        } else if (
            motion_config.control_mode == "joint_trajectory"
        ) {
            control_mode_ = ControlMode::JointTrajectory;
        } else {
            throw std::runtime_error("Invalid control mode");
        }

        if (
            target_source_ != "legacy" &&
            target_source_ != "simulation_marker"
        ) {
            throw std::runtime_error("Invalid target source");
        }

        TrajectoryGenerator::Config trajectory;
        trajectory.speed = motion_config.cartesian.speed;
        trajectory.distance_step =
            trajectory_config.distance_step;
        trajectory.curve = trajectory_config.curve;

        CartesianController::Config cartesian;
        cartesian.gain = motion_config.cartesian.gain;
        cartesian.max_velocity = motion_config.cartesian.speed;
        cartesian.goal_tolerance =
            motion_config.cartesian.tolerance;
        cartesian.replan_threshold =
            trajectory_config.replan_threshold;
        cartesian.jacobian_damping =
            data_module_->getJacobianDamping();
        cartesian.trajectory_enabled =
            trajectory_config.enabled;

        JointController::Config joint;
        joint.gain = motion_config.joint.gain;
        joint.max_velocity = 0.6 * motion_config.joint.speed;
        joint.goal_tolerance = motion_config.joint.tolerance;
        joint.start_tolerance =
            motion_config.joint.start_tolerance;
        joint.max_segment_delta =
            motion_config.joint.max_segment_delta;

        ExecutionMonitor::Config monitoring;
        monitoring.goal_timeout =
            motion_config.monitoring.goal_timeout;
        monitoring.stall_timeout =
            motion_config.monitoring.stall_timeout;
        monitoring.tracking_error_limit =
            motion_config.monitoring.tracking_error_limit;
        monitoring.tracking_error_timeout =
            motion_config.monitoring.tracking_error_timeout;

        joint_feedback_ = std::make_unique<JointFeedback>(
            *robot_kinematics_,
            motion_config.feedback_timeout
        );

        motion_guard_ = std::make_unique<MotionGuard>(
            *robot_kinematics_,
            motion_config.joint.speed
        );

        cartesian_controller_ =
            std::make_unique<CartesianController>(
                *robot_kinematics_,
                cartesian,
                trajectory
            );

        joint_controller_ = std::make_unique<JointController>(
            *robot_kinematics_,
            joint,
            trajectory
        );

        execution_monitor_ =
            std::make_unique<ExecutionMonitor>(monitoring);

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

        failure_reason_publisher_ =
            create_publisher<std_msgs::msg::String>(
                "/motion/failure_reason",
                status_qos
            );

        PublishFailureReason("NONE");
        PublishStatus("IDLE");

        RCLCPP_INFO(
            get_logger(),
            "Control mode: %s | target source: %s",
            motion_config.control_mode.c_str(),
            target_source_.c_str()
        );

    } else if (robot_mode_ == "robot") {
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

        servo_module_ = std::make_unique<ServoModule>(self);

        robot_initializer_ =
            std::make_unique<RobotInitializer>(
                servo_module_.get(),
                data_module_.get()
            );

    } else {
        throw std::runtime_error(
            "Invalid robot mode: " + robot_mode_
        );
    }

    main_timer_ = create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&MotionComponent::MainLoop, this)
    );

    RCLCPP_INFO(get_logger(), "Waiting for joint feedback");
}

void MotionComponent::JointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr message
) {
    if (joint_feedback_) {
        joint_feedback_->update(*message);
    }
}

void MotionComponent::SetCartesianTarget(
    const Eigen::Vector3d& target
) {
    if (
        control_mode_ != ControlMode::Cartesian ||
        !motion_guard_ ||
        motion_guard_->isStopped() ||
        !cartesian_controller_
    ) {
        return;
    }

    if (!cartesian_controller_->setTarget(target)) {
        RCLCPP_ERROR(
            get_logger(),
            "Cartesian target rejected: %s",
            cartesian_controller_->lastError().c_str()
        );

        StopWithError(
            "CONTROL_ERROR",
            "CARTESIAN_TARGET_ERROR"
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
        message->header.frame_id != base_frame_ ||
        message->action != visualization_msgs::msg::Marker::ADD
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

void MotionComponent::JointTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr message
) {
    if (
        control_mode_ != ControlMode::JointTrajectory ||
        !motion_guard_ ||
        motion_guard_->isStopped() ||
        !joint_controller_ ||
        !execution_monitor_
    ) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    if (
        !joint_feedback_->isValid() ||
        !joint_feedback_->isFresh(now)
    ) {
        RCLCPP_WARN(
            get_logger(),
            "Trajectory rejected: feedback unavailable"
        );
        return;
    }

    if (!joint_controller_->setTrajectory(
        *message,
        joint_feedback_->positions(),
        now
    )) {
        RCLCPP_WARN(
            get_logger(),
            "Joint trajectory rejected: %s",
            joint_controller_->lastError().c_str()
        );
        return;
    }

    execution_monitor_->reset(now);

    PublishFailureReason("NONE");
    PublishStatus("EXECUTING");

    RCLCPP_INFO(
        get_logger(),
        "Joint trajectory accepted: %zu waypoints, %.3f s",
        message->points.size(),
        joint_controller_->totalTime()
    );
}

bool MotionComponent::IsMotionActive() const {
    return (
        joint_controller_ && joint_controller_->isActive()
    ) || (
        cartesian_controller_ && cartesian_controller_->isActive()
    );
}

void MotionComponent::CancelMotion() {
    if (joint_controller_) {
        joint_controller_->cancel();
    }

    if (cartesian_controller_) {
        cartesian_controller_->cancel();
    }
}

void MotionComponent::StopWithError(
    const std::string& status,
    const std::string& reason
) {
    if (motion_guard_) {
        motion_guard_->stop();
    }

    CancelMotion();
    StopJointMotion();

    PublishFailureReason(reason);
    PublishStatus(status);

    RCLCPP_ERROR(
        get_logger(),
        "Motion stopped: %s (%s)",
        status.c_str(),
        reason.c_str()
    );
}

void MotionComponent::StopCallback(
    const std_msgs::msg::Bool::SharedPtr message
) {
    if (!motion_guard_) {
        return;
    }

    if (message->data) {
        motion_guard_->stop();

        CancelMotion();
        StopJointMotion();

        // Do not overwrite an active failure reason.
        if (
            last_status_ != "CONTROL_ERROR" &&
            last_status_ != "INVALID_FEEDBACK" &&
            last_status_ != "FEEDBACK_TIMEOUT"
        ) {
            PublishFailureReason("NONE");
            PublishStatus("STOPPED");
        }

        RCLCPP_WARN(
            get_logger(),
            "Motion stopped and trajectory cancelled"
        );

        return;
    }

    if (!motion_guard_->isStopped()) {
        return;
    }

    motion_guard_->release();

    CancelMotion();
    StopJointMotion();

    PublishFailureReason("NONE");
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

void MotionComponent::PublishFailureReason(
    const std::string& reason
) {
    if (reason == last_failure_reason_) {
        return;
    }

    last_failure_reason_ = reason;

    if (!failure_reason_publisher_) {
        return;
    }

    std_msgs::msg::String message;
    message.data = reason;

    failure_reason_publisher_->publish(message);
}

void MotionComponent::StopJointMotion() {
    if (!joint_velocity_publisher_) {
        return;
    }

    std_msgs::msg::Float64MultiArray message;
    message.data.assign(5, 0.0);

    joint_velocity_publisher_->publish(message);
}

bool MotionComponent::FilterJointVelocity(
    const JointVector& velocity,
    JointVector& safe_velocity
) {
    if (
        !joint_feedback_ ||
        !motion_guard_
    ) {
        StopWithError(
            "CONTROL_ERROR",
            "MOTION_COMPONENT_ERROR"
        );
        return false;
    }

    if (
        !joint_feedback_->isValid() ||
        !joint_feedback_->isFresh()
    ) {
        StopWithError(
            "INVALID_FEEDBACK",
            "INVALID_FEEDBACK"
        );
        return false;
    }

    safe_velocity.setZero();

    if (!motion_guard_->filterVelocity(
        velocity,
        joint_feedback_->positions(),
        safe_velocity
    )) {
        StopWithError(
            "CONTROL_ERROR",
            "MOTION_GUARD_ERROR"
        );
        return false;
    }

    if (!safe_velocity.allFinite()) {
        StopWithError(
            "CONTROL_ERROR",
            "INVALID_VELOCITY"
        );
        return false;
    }

    return true;
}

void MotionComponent::PublishFilteredVelocity(
    const JointVector& velocity
) {
    if (!joint_velocity_publisher_) {
        return;
    }

    std_msgs::msg::Float64MultiArray message;
    message.data.resize(5);

    for (int i = 0; i < 5; ++i) {
        message.data[i] = velocity[i];
    }

    joint_velocity_publisher_->publish(message);
}

void MotionComponent::PublishJointVelocity(
    const JointVector& velocity
) {
    JointVector safe_velocity = JointVector::Zero();

    if (!FilterJointVelocity(velocity, safe_velocity)) {
        return;
    }

    PublishFilteredVelocity(safe_velocity);
}

void MotionComponent::RunJointTrajectoryControl(
    const TimePoint& now
) {
    if (!joint_controller_->isActive()) {
        StopJointMotion();
        return;
    }

    const JointVector current_joints =
        joint_feedback_->positions();

    const auto result = joint_controller_->update(
        current_joints,
        now
    );

    switch (result.state) {
        case JointController::State::Running: {
            JointVector safe_velocity = JointVector::Zero();

            if (!FilterJointVelocity(
                result.velocity,
                safe_velocity
            )) {
                return;
            }

            const auto monitor_state = execution_monitor_->update(
                now,
                joint_controller_->totalTime(),
                result.reference_position,
                current_joints,
                safe_velocity,
                joint_controller_->goalTolerance()
            );

            if (monitor_state != ExecutionMonitor::State::Normal) {
                const JointVector error =
                    result.reference_position - current_joints;

                Eigen::Index joint_index = 0;

                const double max_error =
                    error.cwiseAbs().maxCoeff(&joint_index);

                RCLCPP_ERROR(
                    get_logger(),
                    "Execution failure: %s | "
                    "joint=%ld | error=%.3f rad | "
                    "reference=%.3f | actual=%.3f | "
                    "command=%.3f | filtered=%.3f",
                    ExecutionMonitor::stateName(monitor_state),
                    static_cast<long>(joint_index + 1),
                    max_error,
                    result.reference_position[joint_index],
                    current_joints[joint_index],
                    result.velocity[joint_index],
                    safe_velocity[joint_index]
                );

                StopWithError(
                    "CONTROL_ERROR",
                    ExecutionMonitor::stateName(monitor_state)
                );

                return;
            }

            PublishFilteredVelocity(safe_velocity);
            break;
        }

        case JointController::State::Completed:
            StopJointMotion();
            PublishFailureReason("NONE");
            PublishStatus("COMPLETED");
            break;

        case JointController::State::Error:
            RCLCPP_ERROR(
                get_logger(),
                "Joint control error: %s",
                joint_controller_->lastError().c_str()
            );

            StopWithError(
                "CONTROL_ERROR",
                "JOINT_CONTROLLER_ERROR"
            );
            break;

        case JointController::State::Idle:
            StopJointMotion();
            break;
    }
}

void MotionComponent::RunCartesianControl(
    const TimePoint& now
) {
    if (!cartesian_controller_->isActive()) {
        StopJointMotion();
        return;
    }

    const auto result = cartesian_controller_->update(
        joint_feedback_->positions(),
        now
    );

    switch (result.state) {
        case CartesianController::State::Running:
            PublishJointVelocity(result.velocity);

            if (!motion_guard_->isStopped()) {
                PublishStatus("EXECUTING");
            }

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Cartesian tracking error: %.3f mm",
                result.position_error * 1000.0
            );
            break;

        case CartesianController::State::Completed:
            StopJointMotion();
            PublishFailureReason("NONE");
            PublishStatus("COMPLETED");
            break;

        case CartesianController::State::Error:
            RCLCPP_ERROR(
                get_logger(),
                "Cartesian control error: %s",
                cartesian_controller_->lastError().c_str()
            );

            StopWithError(
                "CONTROL_ERROR",
                "CARTESIAN_CONTROLLER_ERROR"
            );
            break;

        case CartesianController::State::Idle:
            StopJointMotion();
            break;
    }
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

    if (motion_guard_->isStopped()) {
        StopJointMotion();
        return;
    }

    const auto now = std::chrono::steady_clock::now();

    if (!joint_feedback_->isValid()) {
        if (IsMotionActive()) {
            StopWithError(
                "INVALID_FEEDBACK",
                "INVALID_FEEDBACK"
            );
        } else {
            StopJointMotion();
            PublishStatus("WAITING_FOR_FEEDBACK");
        }
        return;
    }

    if (!joint_feedback_->isFresh(now)) {
        if (IsMotionActive()) {
            StopWithError(
                "FEEDBACK_TIMEOUT",
                "FEEDBACK_TIMEOUT"
            );
        } else {
            StopJointMotion();
            PublishStatus("WAITING_FOR_FEEDBACK");
        }
        return;
    }

    if (last_status_ == "WAITING_FOR_FEEDBACK") {
        PublishStatus("IDLE");
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