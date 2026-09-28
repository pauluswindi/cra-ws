#ifndef SERVO_COMPONENT_HH_
#define SERVO_COMPONENT_HH_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "std_msgs/msg/float32_multi_array.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "msgs/msg/joint_feedback.hpp"
#include "msgs/msg/set_position.hpp"
#include "msgs/msg/set_velocity.hpp"
#include "msgs/srv/get_joint.hpp"
#include "msgs/srv/set_operating_mode.hpp"
#include "msgs/srv/set_torque.hpp"

namespace servo_driver
{

class ServoComponent : public rclcpp::Node
{
public:
    explicit ServoComponent(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~ServoComponent();

private:
    enum class MotorType
    {
        PH42,
        PH54
    };

    static constexpr char DXL_PORT_NAME[] = "/dev/ttyUSB1";
    static constexpr int DXL_BAUDRATE = 4000000;
    static constexpr float DXL_PROTOCOL_VERSION = 2.0f;

    static constexpr int DXL_ID_COUNT = 2;
    static constexpr std::array<uint8_t, DXL_ID_COUNT> dxl_ids_ = {0, 1};
    static constexpr std::array<MotorType, DXL_ID_COUNT> motor_types_ = {
        MotorType::PH42,
        MotorType::PH54
    };

    static constexpr std::array<int, DXL_ID_COUNT> joint_direction_ = {
        1,
        1
    };

    static constexpr double POSITION_MIN_DEG = -180.0;
    static constexpr double POSITION_MAX_DEG = 180.0;

    static constexpr uint8_t VELOCITY_MODE = 1;
    static constexpr uint8_t POSITION_MODE = 3;

    static constexpr uint16_t ADDR_OPERATING_MODE = 11;
    static constexpr uint16_t ADDR_TORQUE_ENABLE = 512;
    static constexpr uint16_t ADDR_HARDWARE_ERROR_STATUS = 518;
    static constexpr uint16_t ADDR_BUS_WATCHDOG = 546;

    static constexpr uint16_t ADDR_GOAL_VELOCITY = 552;
    static constexpr uint16_t ADDR_GOAL_POSITION = 564;

    static constexpr uint16_t ADDR_PRESENT_CURRENT = 574;
    static constexpr uint16_t ADDR_PRESENT_VELOCITY = 576;
    static constexpr uint16_t ADDR_PRESENT_POSITION = 580;
    static constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 594;

    static constexpr uint16_t SYNC_READ_FEEDBACK_START =
        ADDR_PRESENT_CURRENT;
    static constexpr uint16_t SYNC_READ_FEEDBACK_LENGTH = 10;

    static constexpr uint16_t SYNC_READ_TEMP_START =
        ADDR_PRESENT_TEMPERATURE;
    static constexpr uint16_t SYNC_READ_TEMP_LENGTH = 1;

    static constexpr double PULSES_PER_DEG_PH42 = 303750.0 / 180.0;
    static constexpr double PULSES_PER_DEG_PH54 = 501923.0 / 180.0;

    static constexpr double VELOCITY_RPM_PER_UNIT = 0.01;

    static constexpr uint8_t BUS_WATCHDOG_TICKS = 100;
    static constexpr int BUS_WATCHDOG_TIMEOUT_MS = 2000;
    static constexpr int VELOCITY_COMMAND_TIMEOUT_MS = 250;
    static constexpr int FEEDBACK_PERIOD_MS = 100;
    static constexpr int TEMPERATURE_PERIOD_MS = 1000;
    static constexpr uint8_t TEMPERATURE_WARNING_C = 70;

    static constexpr int MIN_SERVO_ID = 0;
    static constexpr int MAX_SERVO_ID = 1;

    void Init();
    void InitTimerCallback();
    bool InitializeDynamixel();
    void InitializeGroupSync();
    void InitializePublishersSubscribers();

    bool ReadHardwareError(uint8_t id, uint8_t & status);
    void PrintHardwareError(uint8_t id, uint8_t status);
    bool ReadTorqueState(uint8_t id, uint8_t & torque_state);
    bool ReadOperatingMode(uint8_t id, uint8_t & mode);
    bool ConfigureBusWatchdog(uint8_t id, uint8_t ticks);

    bool EnableTorque(uint8_t id, bool enable);
    bool SwitchOperatingMode(uint8_t id, uint8_t mode);

    bool WriteJointPositions(
        const std::array<int32_t, DXL_ID_COUNT> & pulses);
    bool WriteJointVelocities(
        const std::array<int32_t, DXL_ID_COUNT> & velocities);

    bool ReadJointFeedback();
    bool ReadJointTemperatures();

    void FeedbackTimerCallback();
    void TemperatureTimerCallback();

    void SetPositionCallback(
        const msgs::msg::SetPosition::SharedPtr msg);
    void SetVelocityCallback(
        const msgs::msg::SetVelocity::SharedPtr msg);

    void GetJointStateCallback(
        const std::shared_ptr<msgs::srv::GetJoint::Request> request,
        const std::shared_ptr<msgs::srv::GetJoint::Response> response);

    void SetTorqueCallback(
        const std::shared_ptr<msgs::srv::SetTorque::Request> request,
        const std::shared_ptr<msgs::srv::SetTorque::Response> response);

    void SetOperatingModeCallback(
        const std::shared_ptr<msgs::srv::SetOperatingMode::Request> request,
        const std::shared_ptr<msgs::srv::SetOperatingMode::Response> response);

    bool IsValidServoId(int32_t id) const;
    bool IsValidPosition(double deg, int joint_index) const;

    double PulsesPerDegree(int joint_index) const;
    double WrapDegrees(double degrees) const;

    int32_t DegToPulse(double deg, int joint_index) const;
    double PulseToDeg(int32_t pulse, int joint_index) const;

    int32_t RpmToVelocityUnit(double rpm, int joint_index) const;
    double VelocityUnitToRpm(int32_t unit, int joint_index) const;

    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    rclcpp::TimerBase::SharedPtr temperature_timer_;

    rclcpp::Publisher<msgs::msg::JointFeedback>::SharedPtr joint_feedback_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
        joint_temperature_pub_;

    rclcpp::Subscription<msgs::msg::SetPosition>::SharedPtr position_cmd_sub_;
    rclcpp::Subscription<msgs::msg::SetVelocity>::SharedPtr velocity_cmd_sub_;

    rclcpp::Service<msgs::srv::GetJoint>::SharedPtr get_joint_state_srv_;
    rclcpp::Service<msgs::srv::SetTorque>::SharedPtr set_torque_srv_;
    rclcpp::Service<msgs::srv::SetOperatingMode>::SharedPtr
        set_operating_mode_srv_;

    rclcpp::CallbackGroup::SharedPtr service_cb_group_;

    dynamixel::PortHandler * portHandler_ = nullptr;
    dynamixel::PacketHandler * packetHandler_ = nullptr;

    dynamixel::GroupSyncWrite * groupSyncWritePosition_ = nullptr;
    dynamixel::GroupSyncWrite * groupSyncWriteVelocity_ = nullptr;
    dynamixel::GroupSyncRead * groupSyncReadFeedback_ = nullptr;
    dynamixel::GroupSyncRead * groupSyncReadTemperature_ = nullptr;

    std::atomic_bool initialized_{false};
    std::atomic_bool hardware_ready_{false};
    std::atomic_bool velocity_watchdog_stopped_{false};
    std::atomic<uint8_t> current_operating_mode_{VELOCITY_MODE};
    std::atomic<int64_t> last_velocity_command_ns_{0};

    std::array<int32_t, DXL_ID_COUNT> joint_positions_{};
    std::array<int32_t, DXL_ID_COUNT> joint_velocities_{};
    std::array<int16_t, DXL_ID_COUNT> joint_currents_{};
    std::array<uint8_t, DXL_ID_COUNT> joint_temperatures_{};

    std::mutex port_mutex_;
    std::mutex state_mutex_;
};

}  // namespace servo_driver

#endif  // SERVO_COMPONENT_HH_