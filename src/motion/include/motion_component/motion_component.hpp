#ifndef MOTION_COMPONENT_HPP_
#define MOTION_COMPONENT_HPP_

#include "common.hpp"
#include <memory>
#include <atomic>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

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
    void Init();
    void InitTimerCallback();
    void MainLoop();
    void RunInitTest();

    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr main_timer_;

    bool initialized_{false};
    MotionState::State state_ = MotionState::State::Init;

    std::unique_ptr<DataModule> data_module_;
    std::unique_ptr<ServoModule> servo_module_;
    std::unique_ptr<RobotInitializer> robot_initializer_;

    std::atomic<bool> init_thread_running_{false};
};

} // namespace motion

#endif // MOTION_COMPONENT_HPP_