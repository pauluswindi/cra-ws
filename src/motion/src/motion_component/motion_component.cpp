#include "motion_component/motion_component.hpp"
#include <chrono>

namespace motion {

MotionComponent::MotionComponent(const rclcpp::NodeOptions& options)
: Node("motion_component", options),
  initialized_(false)
{
    Init();
    RCLCPP_INFO(get_logger(), "MotionComponent constructed");
}

MotionComponent::~MotionComponent() = default;

void MotionComponent::Init() {
    if (initialized_) {
        RCLCPP_WARN(get_logger(), "MotionComponent already initialized");
        return;
    }
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MotionComponent::InitTimerCallback, this)
    );
}

void MotionComponent::InitTimerCallback() {
    if (initialized_) {
        return;
    }
    if (init_timer_) {
        init_timer_->cancel();
        init_timer_.reset();
    }
    RCLCPP_INFO(get_logger(), "Starting MotionComponent initialization...");
    InitComponent();
    initialized_ = true;
}

void MotionComponent::InitComponent() {
    auto self = shared_from_this();

    data_module_       = std::make_unique<DataModule>(self);
    servo_module_      = std::make_unique<ServoModule>(self);
    robot_initializer_ = std::make_unique<RobotInitializer>(servo_module_.get(), data_module_.get());

    main_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&MotionComponent::MainLoop, this)
    );
    RCLCPP_INFO(get_logger(), "MotionComponent init done");
}

void MotionComponent::MainLoop() {
    switch (state_) {
        case MotionState::State::Init : RunInitTest(); break;
        default: break;
    }
}

void MotionComponent::RunInitTest() {
    if (!init_thread_running_) {
        init_thread_running_ = true;
        std::thread([this]() {
            robot_initializer_->PosRobotInit();
            init_thread_running_ = false;
        }).detach();
    }
}

} // namespace motion

RCLCPP_COMPONENTS_REGISTER_NODE(motion::MotionComponent)