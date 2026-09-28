#include "servo_component.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

namespace servo_driver
{

ServoComponent::ServoComponent(const rclcpp::NodeOptions & options)
    : Node("servo_component", options)
{
    service_cb_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    last_velocity_command_ns_.store(
        this->now().nanoseconds(),
        std::memory_order_relaxed);

    Init();

    RCLCPP_INFO(get_logger(), "ServoComponent constructed");
}

ServoComponent::~ServoComponent()
{
    initialized_.store(false, std::memory_order_relaxed);
    hardware_ready_.store(false, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(port_mutex_);

    if (portHandler_ != nullptr)
    {
        portHandler_->closePort();
    }

    delete groupSyncWritePosition_;
    delete groupSyncWriteVelocity_;
    delete groupSyncReadFeedback_;
    delete groupSyncReadTemperature_;

    groupSyncWritePosition_ = nullptr;
    groupSyncWriteVelocity_ = nullptr;
    groupSyncReadFeedback_ = nullptr;
    groupSyncReadTemperature_ = nullptr;
    portHandler_ = nullptr;
    packetHandler_ = nullptr;

    RCLCPP_INFO(get_logger(), "ServoComponent destroyed");
}

void ServoComponent::Init()
{
    if (initialized_.load(std::memory_order_relaxed))
    {
        return;
    }

    init_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ServoComponent::InitTimerCallback, this));
}

void ServoComponent::InitTimerCallback()
{
    if (init_timer_)
    {
        init_timer_->cancel();
        init_timer_.reset();
    }

    RCLCPP_INFO(get_logger(), "Starting DYNAMIXEL initialization");

    const bool success = InitializeDynamixel();

    InitializePublishersSubscribers();

    hardware_ready_.store(success, std::memory_order_release);
    initialized_.store(true, std::memory_order_release);

    if (success)
    {
        RCLCPP_INFO(
            get_logger(),
            "DYNAMIXEL initialization completed");
    }
    else
    {
        RCLCPP_ERROR(
            get_logger(),
            "DYNAMIXEL initialization failed");
    }
}

bool ServoComponent::InitializeDynamixel()
{
    std::lock_guard<std::mutex> lock(port_mutex_);

    portHandler_ = dynamixel::PortHandler::getPortHandler(DXL_PORT_NAME);
    packetHandler_ =
        dynamixel::PacketHandler::getPacketHandler(DXL_PROTOCOL_VERSION);

    while (rclcpp::ok() && !portHandler_->openPort())
    {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to open %s, retrying",
            DXL_PORT_NAME);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (!rclcpp::ok())
    {
        return false;
    }

    while (rclcpp::ok() &&
           !portHandler_->setBaudRate(DXL_BAUDRATE))
    {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to set baudrate %d, retrying",
            DXL_BAUDRATE);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (!rclcpp::ok())
    {
        return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "Port=%s baudrate=%d protocol=%.1f",
        DXL_PORT_NAME,
        DXL_BAUDRATE,
        DXL_PROTOCOL_VERSION);

    InitializeGroupSync();

    std::array<bool, DXL_ID_COUNT> configured{};

    for (int i = 0; i < DXL_ID_COUNT; ++i)
    {
        const uint8_t id = dxl_ids_[i];
        const char * model =
            motor_types_[i] == MotorType::PH42
                ? "PH42-020-S300-R"
                : "PH54-100-S500-R";

        RCLCPP_INFO(
            get_logger(),
            "Configuring ID %d (%s)",
            id,
            model);

        uint8_t hardware_error = 0;

        if (!ReadHardwareError(id, hardware_error))
        {
            RCLCPP_ERROR(
                get_logger(),
                "ID %d: cannot read Hardware Error Status",
                id);
            continue;
        }

        if (hardware_error != 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "ID %d: hardware error is active, configuration skipped",
                id);
            continue;
        }

        uint8_t torque_state = 0;
        if (ReadTorqueState(id, torque_state))
        {
            RCLCPP_INFO(
                get_logger(),
                "ID %d: Torque Enable=%u",
                id,
                torque_state);
        }

        uint8_t current_mode = 0;
        if (ReadOperatingMode(id, current_mode))
        {
            RCLCPP_INFO(
                get_logger(),
                "ID %d: Operating Mode=%u",
                id,
                current_mode);
        }

        bool motor_configured = false;

        for (int attempt = 1; attempt <= 3; ++attempt)
        {
            RCLCPP_INFO(
                get_logger(),
                "ID %d configuration attempt %d/3",
                id,
                attempt);

            uint8_t dxl_error = 0;

            int result = packetHandler_->write1ByteTxRx(
                portHandler_,
                id,
                ADDR_TORQUE_ENABLE,
                0,
                &dxl_error);

            if (result != COMM_SUCCESS || dxl_error != 0)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "ID %d: Torque OFF failed, result=%d (%s), "
                    "dxl_error=%u (%s)",
                    id,
                    result,
                    packetHandler_->getTxRxResult(result),
                    dxl_error,
                    packetHandler_->getRxPacketError(dxl_error));

                ReadHardwareError(id, hardware_error);
                continue;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(50));

            dxl_error = 0;
            result = packetHandler_->write1ByteTxRx(
                portHandler_,
                id,
                ADDR_OPERATING_MODE,
                VELOCITY_MODE,
                &dxl_error);

            if (result != COMM_SUCCESS || dxl_error != 0)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "ID %d: Operating Mode=1 failed, result=%d (%s), "
                    "dxl_error=%u (%s)",
                    id,
                    result,
                    packetHandler_->getTxRxResult(result),
                    dxl_error,
                    packetHandler_->getRxPacketError(dxl_error));

                ReadHardwareError(id, hardware_error);
                continue;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(50));

            dxl_error = 0;
            result = packetHandler_->write1ByteTxRx(
                portHandler_,
                id,
                ADDR_TORQUE_ENABLE,
                1,
                &dxl_error);

            if (result != COMM_SUCCESS || dxl_error != 0)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "ID %d: Torque ON failed, result=%d (%s), "
                    "dxl_error=%u (%s)",
                    id,
                    result,
                    packetHandler_->getTxRxResult(result),
                    dxl_error,
                    packetHandler_->getRxPacketError(dxl_error));

                ReadHardwareError(id, hardware_error);
                continue;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(50));

            uint8_t verified_torque = 0;
            if (!ReadTorqueState(id, verified_torque) ||
                verified_torque != 1)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "ID %d: Torque Enable verification failed",
                    id);
                ReadHardwareError(id, hardware_error);
                continue;
            }

            motor_configured = true;

            RCLCPP_INFO(
                get_logger(),
                "ID %d configured in Velocity Mode",
                id);
            break;
        }

        configured[i] = motor_configured;

        if (!motor_configured)
        {
            RCLCPP_ERROR(
                get_logger(),
                "ID %d could not be configured",
                id);
        }
    }

    bool all_success = true;

    for (int i = 0; i < DXL_ID_COUNT; ++i)
    {
        if (!configured[i])
        {
            all_success = false;
            continue;
        }

        if (!ConfigureBusWatchdog(dxl_ids_[i], BUS_WATCHDOG_TICKS))
        {
            all_success = false;
            RCLCPP_ERROR(
                get_logger(),
                "ID %d: failed to enable Bus Watchdog",
                dxl_ids_[i]);
        }
    }

    if (!all_success)
    {
        for (int i = 0; i < DXL_ID_COUNT; ++i)
        {
            if (configured[i])
            {
                ConfigureBusWatchdog(dxl_ids_[i], 0);
                packetHandler_->write1ByteTxRx(
                    portHandler_,
                    dxl_ids_[i],
                    ADDR_TORQUE_ENABLE,
                    0,
                    nullptr);
            }
        }

        return false;
    }

    current_operating_mode_.store(
        VELOCITY_MODE,
        std::memory_order_release);

    velocity_watchdog_stopped_.store(
        false,
        std::memory_order_release);

    RCLCPP_INFO(
        get_logger(),
        "Bus Watchdog enabled: %d ms",
        BUS_WATCHDOG_TIMEOUT_MS);

    return true;
}

void ServoComponent::InitializeGroupSync()
{
    groupSyncWritePosition_ =
        new dynamixel::GroupSyncWrite(
            portHandler_,
            packetHandler_,
            ADDR_GOAL_POSITION,
            4);

    groupSyncWriteVelocity_ =
        new dynamixel::GroupSyncWrite(
            portHandler_,
            packetHandler_,
            ADDR_GOAL_VELOCITY,
            4);

    groupSyncReadFeedback_ =
        new dynamixel::GroupSyncRead(
            portHandler_,
            packetHandler_,
            SYNC_READ_FEEDBACK_START,
            SYNC_READ_FEEDBACK_LENGTH);

    groupSyncReadTemperature_ =
        new dynamixel::GroupSyncRead(
            portHandler_,
            packetHandler_,
            SYNC_READ_TEMP_START,
            SYNC_READ_TEMP_LENGTH);
}

void ServoComponent::InitializePublishersSubscribers()
{
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    joint_feedback_pub_ =
        create_publisher<msgs::msg::JointFeedback>(
            "/servo/joint_feedback",
            qos);

    joint_temperature_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>(
            "/servo/joint_temperatures",
            qos);

    position_cmd_sub_ =
        create_subscription<msgs::msg::SetPosition>(
            "/servo/set_position",
            qos,
            std::bind(
                &ServoComponent::SetPositionCallback,
                this,
                std::placeholders::_1));

    velocity_cmd_sub_ =
        create_subscription<msgs::msg::SetVelocity>(
            "/servo/set_velocity",
            qos,
            std::bind(
                &ServoComponent::SetVelocityCallback,
                this,
                std::placeholders::_1));

    const rmw_qos_profile_t service_qos =
        rmw_qos_profile_services_default;

    get_joint_state_srv_ =
        create_service<msgs::srv::GetJoint>(
            "/servo/get_joint",
            std::bind(
                &ServoComponent::GetJointStateCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            service_qos,
            service_cb_group_);

    set_torque_srv_ =
        create_service<msgs::srv::SetTorque>(
            "/servo/set_torque",
            std::bind(
                &ServoComponent::SetTorqueCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            service_qos,
            service_cb_group_);

    set_operating_mode_srv_ =
        create_service<msgs::srv::SetOperatingMode>(
            "/servo/set_operating_mode",
            std::bind(
                &ServoComponent::SetOperatingModeCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            service_qos,
            service_cb_group_);

    feedback_timer_ = create_wall_timer(
        std::chrono::milliseconds(FEEDBACK_PERIOD_MS),
        std::bind(&ServoComponent::FeedbackTimerCallback, this));

    temperature_timer_ = create_wall_timer(
        std::chrono::milliseconds(TEMPERATURE_PERIOD_MS),
        std::bind(&ServoComponent::TemperatureTimerCallback, this));

    RCLCPP_INFO(get_logger(), "ROS interfaces initialized");
}

void ServoComponent::PrintHardwareError(
    uint8_t id,
    uint8_t status)
{
    if (status == 0)
    {
        RCLCPP_DEBUG(
            get_logger(),
            "ID %d: Hardware Error Status=0",
            id);
        return;
    }

    RCLCPP_ERROR(
        get_logger(),
        "ID %d: Hardware Error Status=%u (0x%02X)",
        id,
        status,
        status);

    if (status & 0x01)
    {
        RCLCPP_ERROR(get_logger(), "ID %d: Input Voltage Error", id);
    }
    if (status & 0x02)
    {
        RCLCPP_ERROR(get_logger(), "ID %d: Motor Hall Sensor Error", id);
    }
    if (status & 0x04)
    {
        RCLCPP_ERROR(get_logger(), "ID %d: Overheating Error", id);
    }
    if (status & 0x08)
    {
        RCLCPP_ERROR(get_logger(), "ID %d: Motor Encoder Error", id);
    }
    if (status & 0x10)
    {
        RCLCPP_ERROR(get_logger(), "ID %d: Electrical Shock Error", id);
    }
    if (status & 0x20)
    {
        RCLCPP_ERROR(get_logger(), "ID %d: Overload Error", id);
    }
    if (status & 0x80)
    {
        RCLCPP_ERROR(get_logger(), "ID %d: Alert Bit", id);
    }
}

bool ServoComponent::ReadHardwareError(
    uint8_t id,
    uint8_t & status)
{
    status = 0;
    uint8_t dxl_error = 0;

    const int result = packetHandler_->read1ByteTxRx(
        portHandler_,
        id,
        ADDR_HARDWARE_ERROR_STATUS,
        &status,
        &dxl_error);

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "ID %d: Hardware Error read failed, result=%d (%s), "
            "dxl_error=%u (%s)",
            id,
            result,
            packetHandler_->getTxRxResult(result),
            dxl_error,
            packetHandler_->getRxPacketError(dxl_error));
        return false;
    }

    PrintHardwareError(id, status);
    return true;
}

bool ServoComponent::ReadTorqueState(
    uint8_t id,
    uint8_t & torque_state)
{
    uint8_t dxl_error = 0;

    const int result = packetHandler_->read1ByteTxRx(
        portHandler_,
        id,
        ADDR_TORQUE_ENABLE,
        &torque_state,
        &dxl_error);

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "ID %d: Torque read failed, result=%d (%s), "
            "dxl_error=%u (%s)",
            id,
            result,
            packetHandler_->getTxRxResult(result),
            dxl_error,
            packetHandler_->getRxPacketError(dxl_error));
        return false;
    }

    return true;
}

bool ServoComponent::ReadOperatingMode(
    uint8_t id,
    uint8_t & mode)
{
    uint8_t dxl_error = 0;

    const int result = packetHandler_->read1ByteTxRx(
        portHandler_,
        id,
        ADDR_OPERATING_MODE,
        &mode,
        &dxl_error);

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "ID %d: Operating Mode read failed, result=%d (%s), "
            "dxl_error=%u (%s)",
            id,
            result,
            packetHandler_->getTxRxResult(result),
            dxl_error,
            packetHandler_->getRxPacketError(dxl_error));
        return false;
    }

    return true;
}

bool ServoComponent::ConfigureBusWatchdog(
    uint8_t id,
    uint8_t ticks)
{
    uint8_t dxl_error = 0;

    int result = packetHandler_->write1ByteTxRx(
        portHandler_,
        id,
        ADDR_BUS_WATCHDOG,
        0,
        &dxl_error);

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "ID %d: failed to clear Bus Watchdog, result=%d (%s), "
            "dxl_error=%u (%s)",
            id,
            result,
            packetHandler_->getTxRxResult(result),
            dxl_error,
            packetHandler_->getRxPacketError(dxl_error));
        return false;
    }

    if (ticks == 0)
    {
        return true;
    }

    dxl_error = 0;
    result = packetHandler_->write1ByteTxRx(
        portHandler_,
        id,
        ADDR_BUS_WATCHDOG,
        ticks,
        &dxl_error);

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "ID %d: failed to set Bus Watchdog=%u, result=%d (%s), "
            "dxl_error=%u (%s)",
            id,
            ticks,
            result,
            packetHandler_->getTxRxResult(result),
            dxl_error,
            packetHandler_->getRxPacketError(dxl_error));
        return false;
    }

    return true;
}

bool ServoComponent::EnableTorque(
    uint8_t id,
    bool enable)
{
    if (!hardware_ready_.load(std::memory_order_acquire))
    {
        RCLCPP_WARN(
            get_logger(),
            "Torque command ignored: hardware is not ready");
        return false;
    }

    std::lock_guard<std::mutex> lock(port_mutex_);

    uint8_t dxl_error = 0;

    const int result = packetHandler_->write1ByteTxRx(
        portHandler_,
        id,
        ADDR_TORQUE_ENABLE,
        enable ? 1 : 0,
        &dxl_error);

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "ID %d: failed to %s torque, result=%d (%s), "
            "dxl_error=%u (%s)",
            id,
            enable ? "enable" : "disable",
            result,
            packetHandler_->getTxRxResult(result),
            dxl_error,
            packetHandler_->getRxPacketError(dxl_error));

        uint8_t hardware_error = 0;
        ReadHardwareError(id, hardware_error);
        return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "ID %d: Torque %s",
        id,
        enable ? "ON" : "OFF");

    return true;
}

bool ServoComponent::SwitchOperatingMode(
    uint8_t id,
    uint8_t mode)
{
    if (!hardware_ready_.load(std::memory_order_acquire))
    {
        RCLCPP_WARN(
            get_logger(),
            "Mode change ignored: hardware is not ready");
        return false;
    }

    if (mode != VELOCITY_MODE && mode != POSITION_MODE)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Unsupported operating mode: %u",
            mode);
        return false;
    }

    std::lock_guard<std::mutex> lock(port_mutex_);

    uint8_t dxl_error = 0;

    int result = packetHandler_->write1ByteTxRx(
        portHandler_,
        id,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "ID %d: failed to disable torque before mode change, "
            "result=%d (%s), dxl_error=%u (%s)",
            id,
            result,
            packetHandler_->getTxRxResult(result),
            dxl_error,
            packetHandler_->getRxPacketError(dxl_error));

        uint8_t hardware_error = 0;
        ReadHardwareError(id, hardware_error);
        return false;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(50));

    dxl_error = 0;
    result = packetHandler_->write1ByteTxRx(
        portHandler_,
        id,
        ADDR_OPERATING_MODE,
        mode,
        &dxl_error);

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "ID %d: failed to set Operating Mode=%u, "
            "result=%d (%s), dxl_error=%u (%s)",
            id,
            mode,
            result,
            packetHandler_->getTxRxResult(result),
            dxl_error,
            packetHandler_->getRxPacketError(dxl_error));

        uint8_t hardware_error = 0;
        ReadHardwareError(id, hardware_error);
        return false;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(50));

    dxl_error = 0;
    result = packetHandler_->write1ByteTxRx(
        portHandler_,
        id,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    if (result != COMM_SUCCESS || dxl_error != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "ID %d: failed to re-enable torque, result=%d (%s), "
            "dxl_error=%u (%s)",
            id,
            result,
            packetHandler_->getTxRxResult(result),
            dxl_error,
            packetHandler_->getRxPacketError(dxl_error));

        uint8_t hardware_error = 0;
        ReadHardwareError(id, hardware_error);
        return false;
    }

    current_operating_mode_.store(
        mode,
        std::memory_order_release);

    if (mode == VELOCITY_MODE)
    {
        velocity_watchdog_stopped_.store(
            false,
            std::memory_order_release);

        last_velocity_command_ns_.store(
            this->now().nanoseconds(),
            std::memory_order_release);
    }

    RCLCPP_INFO(
        get_logger(),
        "ID %d: Operating Mode=%u",
        id,
        mode);

    return true;
}

void ServoComponent::SetPositionCallback(
    const msgs::msg::SetPosition::SharedPtr msg)
{
    if (!hardware_ready_.load(std::memory_order_acquire) ||
        current_operating_mode_.load(std::memory_order_acquire) !=
            POSITION_MODE)
    {
        RCLCPP_WARN(
            get_logger(),
            "Position command ignored: hardware is not ready or "
            "node is not in Position Mode");
        return;
    }

    if (static_cast<int>(msg->positions.size()) != DXL_ID_COUNT)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Position array size mismatch");
        return;
    }

    std::array<int32_t, DXL_ID_COUNT> pulses{};

    for (int i = 0; i < DXL_ID_COUNT; ++i)
    {
        if (!IsValidPosition(msg->positions[i], i))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Joint %d position %.3f deg is outside %.1f..%.1f deg",
                i,
                msg->positions[i],
                POSITION_MIN_DEG,
                POSITION_MAX_DEG);
            return;
        }

        pulses[i] = DegToPulse(msg->positions[i], i);
    }

    if (!WriteJointPositions(pulses))
    {
        RCLCPP_ERROR(
            get_logger(),
            "Position command failed");
    }
}

void ServoComponent::SetVelocityCallback(
    const msgs::msg::SetVelocity::SharedPtr msg)
{
    if (!hardware_ready_.load(std::memory_order_acquire) ||
        current_operating_mode_.load(std::memory_order_acquire) !=
            VELOCITY_MODE)
    {
        RCLCPP_WARN(
            get_logger(),
            "Velocity command ignored: hardware is not ready or "
            "node is not in Velocity Mode");
        return;
    }

    if (static_cast<int>(msg->velocities.size()) != DXL_ID_COUNT)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Velocity array size mismatch");
        return;
    }

    std::array<int32_t, DXL_ID_COUNT> raw_velocities{};

    for (int i = 0; i < DXL_ID_COUNT; ++i)
    {
        if (!std::isfinite(msg->velocities[i]))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Joint %d velocity is not finite",
                i);
            return;
        }

        raw_velocities[i] =
            RpmToVelocityUnit(msg->velocities[i], i);
    }

    if (!WriteJointVelocities(raw_velocities))
    {
        RCLCPP_ERROR(
            get_logger(),
            "Velocity command failed");
        return;
    }

    last_velocity_command_ns_.store(
        this->now().nanoseconds(),
        std::memory_order_release);

    velocity_watchdog_stopped_.store(
        false,
        std::memory_order_release);
}

void ServoComponent::GetJointStateCallback(
    const std::shared_ptr<msgs::srv::GetJoint::Request> request,
    const std::shared_ptr<msgs::srv::GetJoint::Response> response)
{
    if (!IsValidServoId(request->id))
    {
        response->success = false;
        response->message = "Invalid servo ID";
        return;
    }

    auto it = std::find(
        dxl_ids_.begin(),
        dxl_ids_.end(),
        static_cast<uint8_t>(request->id));

    if (it == dxl_ids_.end())
    {
        response->success = false;
        response->message = "Servo ID not configured";
        return;
    }

    const int index =
        static_cast<int>(std::distance(dxl_ids_.begin(), it));

    std::lock_guard<std::mutex> lock(state_mutex_);

    response->success = true;
    response->message = "Success";
    response->position = PulseToDeg(joint_positions_[index], index);
    response->velocity =
        VelocityUnitToRpm(joint_velocities_[index], index);
    response->current =
        static_cast<double>(joint_currents_[index]);
}

void ServoComponent::SetTorqueCallback(
    const std::shared_ptr<msgs::srv::SetTorque::Request> request,
    const std::shared_ptr<msgs::srv::SetTorque::Response> response)
{
    if (!IsValidServoId(request->id))
    {
        response->success = false;
        return;
    }

    response->success =
        EnableTorque(
            static_cast<uint8_t>(request->id),
            request->enable);
}

void ServoComponent::SetOperatingModeCallback(
    const std::shared_ptr<msgs::srv::SetOperatingMode::Request> request,
    const std::shared_ptr<msgs::srv::SetOperatingMode::Response> response)
{
    if (!IsValidServoId(request->id))
    {
        response->success = false;
        return;
    }

    response->success =
        SwitchOperatingMode(
            static_cast<uint8_t>(request->id),
            static_cast<uint8_t>(request->mode));
}

bool ServoComponent::WriteJointPositions(
    const std::array<int32_t, DXL_ID_COUNT> & pulses)
{
    std::lock_guard<std::mutex> lock(port_mutex_);

    if (groupSyncWritePosition_ == nullptr)
    {
        return false;
    }

    groupSyncWritePosition_->clearParam();

    bool all_added = true;

    for (int i = 0; i < DXL_ID_COUNT; ++i)
    {
        const int32_t target = pulses[i];

        uint8_t param[4] = {
            DXL_LOBYTE(DXL_LOWORD(target)),
            DXL_HIBYTE(DXL_LOWORD(target)),
            DXL_LOBYTE(DXL_HIWORD(target)),
            DXL_HIBYTE(DXL_HIWORD(target))
        };

        if (!groupSyncWritePosition_->addParam(
                dxl_ids_[i],
                param))
        {
            all_added = false;
            RCLCPP_ERROR(
                get_logger(),
                "Failed to add position for ID %d",
                dxl_ids_[i]);
        }
    }

    if (!all_added)
    {
        groupSyncWritePosition_->clearParam();
        return false;
    }

    const int result = groupSyncWritePosition_->txPacket();
    groupSyncWritePosition_->clearParam();

    if (result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Position sync write failed, result=%d (%s)",
            result,
            packetHandler_->getTxRxResult(result));
        return false;
    }

    return true;
}

bool ServoComponent::WriteJointVelocities(
    const std::array<int32_t, DXL_ID_COUNT> & velocities)
{
    std::lock_guard<std::mutex> lock(port_mutex_);

    if (groupSyncWriteVelocity_ == nullptr)
    {
        return false;
    }

    groupSyncWriteVelocity_->clearParam();

    bool all_added = true;

    for (int i = 0; i < DXL_ID_COUNT; ++i)
    {
        const int32_t target = velocities[i];

        uint8_t param[4] = {
            DXL_LOBYTE(DXL_LOWORD(target)),
            DXL_HIBYTE(DXL_LOWORD(target)),
            DXL_LOBYTE(DXL_HIWORD(target)),
            DXL_HIBYTE(DXL_HIWORD(target))
        };

        if (!groupSyncWriteVelocity_->addParam(
                dxl_ids_[i],
                param))
        {
            all_added = false;
            RCLCPP_ERROR(
                get_logger(),
                "Failed to add velocity for ID %d",
                dxl_ids_[i]);
        }
    }

    if (!all_added)
    {
        groupSyncWriteVelocity_->clearParam();
        return false;
    }

    const int result = groupSyncWriteVelocity_->txPacket();
    groupSyncWriteVelocity_->clearParam();

    if (result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Velocity sync write failed, result=%d (%s)",
            result,
            packetHandler_->getTxRxResult(result));
        return false;
    }

    return true;
}

bool ServoComponent::ReadJointFeedback()
{
    std::lock_guard<std::mutex> port_lock(port_mutex_);

    if (groupSyncReadFeedback_ == nullptr)
    {
        return false;
    }

    groupSyncReadFeedback_->clearParam();

    for (const uint8_t id : dxl_ids_)
    {
        if (!groupSyncReadFeedback_->addParam(id))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to add feedback read for ID %d",
                id);
            groupSyncReadFeedback_->clearParam();
            return false;
        }
    }

    const int result = groupSyncReadFeedback_->txRxPacket();

    if (result != COMM_SUCCESS)
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Feedback read failed, result=%d (%s)",
            result,
            packetHandler_->getTxRxResult(result));
        groupSyncReadFeedback_->clearParam();
        return false;
    }

    std::lock_guard<std::mutex> state_lock(state_mutex_);

    for (int i = 0; i < DXL_ID_COUNT; ++i)
    {
        const uint8_t id = dxl_ids_[i];

        if (!groupSyncReadFeedback_->isAvailable(
                id,
                ADDR_PRESENT_CURRENT,
                SYNC_READ_FEEDBACK_LENGTH))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Feedback not available for ID %d",
                id);
            continue;
        }

        joint_positions_[i] =
            static_cast<int32_t>(
                groupSyncReadFeedback_->getData(
                    id,
                    ADDR_PRESENT_POSITION,
                    4));

        joint_velocities_[i] =
            static_cast<int32_t>(
                groupSyncReadFeedback_->getData(
                    id,
                    ADDR_PRESENT_VELOCITY,
                    4));

        joint_currents_[i] =
            static_cast<int16_t>(
                groupSyncReadFeedback_->getData(
                    id,
                    ADDR_PRESENT_CURRENT,
                    2));
    }

    groupSyncReadFeedback_->clearParam();
    return true;
}

bool ServoComponent::ReadJointTemperatures()
{
    std::lock_guard<std::mutex> port_lock(port_mutex_);

    if (groupSyncReadTemperature_ == nullptr)
    {
        return false;
    }

    groupSyncReadTemperature_->clearParam();

    for (const uint8_t id : dxl_ids_)
    {
        if (!groupSyncReadTemperature_->addParam(id))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to add temperature read for ID %d",
                id);
            groupSyncReadTemperature_->clearParam();
            return false;
        }
    }

    const int result = groupSyncReadTemperature_->txRxPacket();

    if (result != COMM_SUCCESS)
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Temperature read failed, result=%d (%s)",
            result,
            packetHandler_->getTxRxResult(result));
        groupSyncReadTemperature_->clearParam();
        return false;
    }

    std::lock_guard<std::mutex> state_lock(state_mutex_);

    for (int i = 0; i < DXL_ID_COUNT; ++i)
    {
        const uint8_t id = dxl_ids_[i];

        if (!groupSyncReadTemperature_->isAvailable(
                id,
                SYNC_READ_TEMP_START,
                SYNC_READ_TEMP_LENGTH))
        {
            continue;
        }

        joint_temperatures_[i] =
            static_cast<uint8_t>(
                groupSyncReadTemperature_->getData(
                    id,
                    ADDR_PRESENT_TEMPERATURE,
                    1));

        if (joint_temperatures_[i] >= TEMPERATURE_WARNING_C)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "ID %d temperature is %u C",
                id,
                joint_temperatures_[i]);
        }
    }

    groupSyncReadTemperature_->clearParam();
    return true;
}

void ServoComponent::FeedbackTimerCallback()
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return;
    }

    ReadJointFeedback();

    if (hardware_ready_.load(std::memory_order_acquire) &&
        current_operating_mode_.load(std::memory_order_acquire) ==
            VELOCITY_MODE)
    {
        const int64_t now_ns = this->now().nanoseconds();
        const int64_t last_ns =
            last_velocity_command_ns_.load(
                std::memory_order_acquire);

        const bool timed_out =
            (now_ns - last_ns) >
            static_cast<int64_t>(VELOCITY_COMMAND_TIMEOUT_MS) * 1000000LL;

        if (timed_out &&
            !velocity_watchdog_stopped_.exchange(
                true,
                std::memory_order_acq_rel))
        {
            std::array<int32_t, DXL_ID_COUNT> zero_velocities{};

            RCLCPP_WARN(
                get_logger(),
                "Velocity command timeout; stopping joints");

            WriteJointVelocities(zero_velocities);
        }
    }

    msgs::msg::JointFeedback msg;
    msg.header.stamp = this->now();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        for (int i = 0; i < DXL_ID_COUNT; ++i)
        {
            msg.name.push_back(
                "joint_" + std::to_string(dxl_ids_[i]));

            msg.position.push_back(
                PulseToDeg(joint_positions_[i], i));

            msg.velocity.push_back(
                VelocityUnitToRpm(
                    joint_velocities_[i],
                    i));

            msg.current.push_back(
                static_cast<double>(joint_currents_[i]));
        }
    }

    joint_feedback_pub_->publish(msg);
}

void ServoComponent::TemperatureTimerCallback()
{
    if (!initialized_.load(std::memory_order_acquire))
    {
        return;
    }

    ReadJointTemperatures();

    std_msgs::msg::Float32MultiArray msg;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        for (const uint8_t temperature : joint_temperatures_)
        {
            msg.data.push_back(
                static_cast<float>(temperature));
        }
    }

    joint_temperature_pub_->publish(msg);
}

bool ServoComponent::IsValidServoId(int32_t id) const
{
    return id >= MIN_SERVO_ID && id <= MAX_SERVO_ID;
}

bool ServoComponent::IsValidPosition(
    double deg,
    int joint_index) const
{
    if (joint_index < 0 || joint_index >= DXL_ID_COUNT)
    {
        return false;
    }

    return std::isfinite(deg) &&
           deg >= POSITION_MIN_DEG &&
           deg <= POSITION_MAX_DEG;
}

double ServoComponent::PulsesPerDegree(int joint_index) const
{
    if (joint_index < 0 || joint_index >= DXL_ID_COUNT)
    {
        return 0.0;
    }

    return motor_types_[joint_index] == MotorType::PH54
        ? PULSES_PER_DEG_PH54
        : PULSES_PER_DEG_PH42;
}

double ServoComponent::WrapDegrees(double degrees) const
{
    if (!std::isfinite(degrees))
    {
        return 0.0;
    }

    degrees = std::fmod(degrees + 180.0, 360.0);

    if (degrees < 0.0)
    {
        degrees += 360.0;
    }

    return degrees - 180.0;
}

int32_t ServoComponent::DegToPulse(
    double deg,
    int joint_index) const
{
    const double factor = PulsesPerDegree(joint_index);

    if (factor <= 0.0)
    {
        return 0;
    }

    const double pulse =
        deg *
        factor *
        static_cast<double>(joint_direction_[joint_index]);

    if (pulse > static_cast<double>(std::numeric_limits<int32_t>::max()))
    {
        return std::numeric_limits<int32_t>::max();
    }

    if (pulse < static_cast<double>(std::numeric_limits<int32_t>::min()))
    {
        return std::numeric_limits<int32_t>::min();
    }

    return static_cast<int32_t>(std::lround(pulse));
}

double ServoComponent::PulseToDeg(
    int32_t pulse,
    int joint_index) const
{
    const double factor = PulsesPerDegree(joint_index);

    if (factor <= 0.0)
    {
        return 0.0;
    }

    const double degrees =
        static_cast<double>(pulse) /
        factor *
        static_cast<double>(joint_direction_[joint_index]);

    return WrapDegrees(degrees);
}

int32_t ServoComponent::RpmToVelocityUnit(
    double rpm,
    int joint_index) const
{
    if (joint_index < 0 || joint_index >= DXL_ID_COUNT ||
        !std::isfinite(rpm))
    {
        return 0;
    }

    const double unit =
        rpm /
        VELOCITY_RPM_PER_UNIT *
        static_cast<double>(joint_direction_[joint_index]);

    if (unit > static_cast<double>(std::numeric_limits<int32_t>::max()))
    {
        return std::numeric_limits<int32_t>::max();
    }

    if (unit < static_cast<double>(std::numeric_limits<int32_t>::min()))
    {
        return std::numeric_limits<int32_t>::min();
    }

    return static_cast<int32_t>(std::lround(unit));
}

double ServoComponent::VelocityUnitToRpm(
    int32_t unit,
    int joint_index) const
{
    if (joint_index < 0 || joint_index >= DXL_ID_COUNT)
    {
        return 0.0;
    }

    return static_cast<double>(unit) *
           VELOCITY_RPM_PER_UNIT *
           static_cast<double>(joint_direction_[joint_index]);
}

}  // namespace servo_driver

RCLCPP_COMPONENTS_REGISTER_NODE(servo_driver::ServoComponent)