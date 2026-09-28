#ifndef MODULES_SERVO_MODULE_HPP_
#define MODULES_SERVO_MODULE_HPP_

#include "common.hpp"
#include <mutex>
#include <array>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32_multi_array.hpp> 

// New Messages & Services
#include "msgs/msg/set_position.hpp"
#include "msgs/msg/set_velocity.hpp"
#include "msgs/msg/joint_feedback.hpp"
#include "msgs/srv/set_torque.hpp"
#include "msgs/srv/set_operating_mode.hpp"
#include "msgs/srv/get_joint.hpp"

// Constants matching servo_component
constexpr int kServoCount = 2;
constexpr uint8_t kModePosition = 3;
constexpr uint8_t kModeVelocity = 1;

class ServoModule {
public:
    explicit ServoModule(const rclcpp::Node::SharedPtr& node);
    ~ServoModule();

    //Command Methods (Publishers)
    void SetJointPosition(const std::array<double, kServoCount>& positions_deg);
    void SetVelocity(const std::array<double, kServoCount>& velocities_rpm);
    
    //Mode & Torque Control (Services)
    bool SetTorque(uint8_t id, bool enable);
    bool SetOperatingMode(uint8_t id, uint8_t mode);

    //Feedback Methods (From Subscription - Fast & Non-blocking)
    double GetJointPosition(uint8_t id); // Returns degrees
    double GetJointVelocity(uint8_t id); // Returns RPM
    double GetJointCurrent(uint8_t id);  // Returns mA
    double GetJointTemperature(uint8_t id); // Returns Celsius

    std::array<double, kServoCount> GetAllJointPositions();
    std::array<double, kServoCount> GetAllJointVelocities();

    rclcpp::Logger GetLogger() const { return node_->get_logger(); }

private:
    // Callback for real-time feedback
    void FeedbackCallback(const msgs::msg::JointFeedback::SharedPtr msg);
    void TemperatureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // Service Call Helpers
    bool CallServiceSetTorque(uint8_t id, bool enable);
    bool CallServiceSetMode(uint8_t id, uint8_t mode);

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // Publishers
    rclcpp::Publisher<msgs::msg::SetPosition>::SharedPtr position_pub_;
    rclcpp::Publisher<msgs::msg::SetVelocity>::SharedPtr velocity_pub_;

    // Subscribers
    rclcpp::Subscription<msgs::msg::JointFeedback>::SharedPtr feedback_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr temperature_sub_;

    // Clients
    rclcpp::Client<msgs::srv::SetTorque>::SharedPtr torque_client_;
    rclcpp::Client<msgs::srv::SetOperatingMode>::SharedPtr mode_client_;
    rclcpp::Client<msgs::srv::GetJoint>::SharedPtr get_joint_client_;

    // Thread-safe Cache for Feedback Data
    mutable std::mutex data_mutex_;
    std::array<double, kServoCount> cached_positions_;
    std::array<double, kServoCount> cached_velocities_;
    std::array<double, kServoCount> cached_currents_;
    std::array<double, kServoCount> cached_temperatures_;
    bool feedback_received_ = false;

    static constexpr int kServiceTimeoutSec = 2;
};

#endif // MODULES_SERVO_MODULE_HPP_