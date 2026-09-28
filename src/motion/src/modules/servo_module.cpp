#include "modules/servo_module.hpp"
#include <algorithm>
#include <chrono>

ServoModule::ServoModule(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    // Use a Reentrant callback group so service calls do not block subscription callbacks
    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    // QoS for commands (reliable, small queue to keep the latest data)
    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    // QoS for feedback (reliable, slightly larger queue to handle bursts)
    auto feedback_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Initialize caches with zero values
    cached_positions_.fill(0.0);
    cached_velocities_.fill(0.0);
    cached_currents_.fill(0.0);
    cached_temperatures_.fill(0.0);

    // Publishers for sending commands to the servo component
    position_pub_ = node_->create_publisher<msgs::msg::SetPosition>("/servo/set_position", cmd_qos);
    velocity_pub_ = node_->create_publisher<msgs::msg::SetVelocity>("/servo/set_velocity", cmd_qos);

    // Subscribers for receiving real-time feedback from the servo component
    feedback_sub_ = node_->create_subscription<msgs::msg::JointFeedback>(
        "/servo/joint_feedback", feedback_qos,
        std::bind(&ServoModule::FeedbackCallback, this, std::placeholders::_1));
    
    temperature_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/servo/joint_temperatures", feedback_qos,
        std::bind(&ServoModule::TemperatureCallback, this, std::placeholders::_1));

    torque_client_ = node_->create_client<msgs::srv::SetTorque>(
        "/servo/set_torque");

    mode_client_ = node_->create_client<msgs::srv::SetOperatingMode>(
        "/servo/set_operating_mode");

    get_joint_client_ = node_->create_client<msgs::srv::GetJoint>(
        "/servo/get_joint", rmw_qos_profile_services_default, cb_group_); 

    RCLCPP_INFO(node_->get_logger(), "ServoModule initialized with new API using Degrees and RPM.");

    RCLCPP_INFO(node_->get_logger(), "ServoModule initialized with new API (Degrees/RPM/mA).");
}

ServoModule::~ServoModule() {
    // Shared pointers clean themselves up automatically,
    // but logging can be added here if needed.
}

// Callbacks (real-time cache update)

void ServoModule::FeedbackCallback(const msgs::msg::JointFeedback::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Ensure the data size matches the expected number of servos
    if (static_cast<int>(msg->position.size()) == kServoCount) {
        for (int i = 0; i < kServoCount; ++i) {
            cached_positions_[i] = msg->position[i];   // Already in degrees
            cached_velocities_[i] = msg->velocity[i];  // Already in RPM
            cached_currents_[i] = msg->current[i];     // Already in mA
        }
        feedback_received_ = true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "Mismatched feedback size: expected %d, got %zu", 
                    kServoCount, msg->position.size());
    }
}

void ServoModule::TemperatureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (static_cast<int>(msg->data.size()) == kServoCount) {
        for (int i = 0; i < kServoCount; ++i) {
            cached_temperatures_[i] = static_cast<double>(msg->data[i]);
        }
    }
}

// Command methods (publishers)

void ServoModule::SetJointPosition(const std::array<double, kServoCount>& positions_deg) {
    auto msg = std::make_shared<msgs::msg::SetPosition>();
    msg->header.stamp = node_->now();
    msg->positions.assign(positions_deg.begin(), positions_deg.end());
    position_pub_->publish(*msg);
}

void ServoModule::SetVelocity(const std::array<double, kServoCount>& velocities_rpm) {
    auto msg = std::make_shared<msgs::msg::SetVelocity>();
    msg->header.stamp = node_->now();
    msg->velocities.assign(velocities_rpm.begin(), velocities_rpm.end());
    velocity_pub_->publish(*msg);
}

// Control methods (services)

bool ServoModule::SetTorque(uint8_t id, bool enable) {
    return CallServiceSetTorque(id, enable);
}

bool ServoModule::SetOperatingMode(uint8_t id, uint8_t mode) {
    return CallServiceSetMode(id, mode);
}

bool ServoModule::CallServiceSetTorque(uint8_t id, bool enable) {
    if (!torque_client_->wait_for_service(std::chrono::seconds(kServiceTimeoutSec))) {
        RCLCPP_ERROR(node_->get_logger(), "SetTorque service not available");
        return false;
    }
    
    auto request = std::make_shared<msgs::srv::SetTorque::Request>();
    request->id = id;
    request->enable = enable;
    
    auto future = torque_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(kServiceTimeoutSec)) == std::future_status::ready) {
        return future.get()->success;
    }
    
    RCLCPP_ERROR(node_->get_logger(), "SetTorque service call timed out for ID %d", id);
    return false;
}

bool ServoModule::CallServiceSetMode(uint8_t id, uint8_t mode) {
    if (!mode_client_->wait_for_service(std::chrono::seconds(kServiceTimeoutSec))) {
        RCLCPP_ERROR(node_->get_logger(), "SetOperatingMode service not available");
        return false;
    }
    
    auto request = std::make_shared<msgs::srv::SetOperatingMode::Request>();
    request->id = id;
    request->mode = mode;
    
    auto future = mode_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(kServiceTimeoutSec)) == std::future_status::ready) {
        return future.get()->success;
    }
    
    RCLCPP_ERROR(node_->get_logger(), "SetOperatingMode service call timed out for ID %d", id);
    return false;
}

// Feedback methods (fast, non-blocking cache read)

double ServoModule::GetJointPosition(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (id < kServoCount) {
        return cached_positions_[id];
    }
    RCLCPP_WARN(node_->get_logger(), "Invalid joint ID requested: %d", id);
    return 0.0;
}

double ServoModule::GetJointVelocity(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (id < kServoCount) {
        return cached_velocities_[id];
    }
    return 0.0;
}

double ServoModule::GetJointCurrent(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (id < kServoCount) {
        return cached_currents_[id];
    }
    return 0.0;
}

double ServoModule::GetJointTemperature(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (id < kServoCount) {
        return cached_temperatures_[id];
    }
    return 0.0;
}

std::array<double, kServoCount> ServoModule::GetAllJointPositions() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return cached_positions_;
}

std::array<double, kServoCount> ServoModule::GetAllJointVelocities() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return cached_velocities_;
}