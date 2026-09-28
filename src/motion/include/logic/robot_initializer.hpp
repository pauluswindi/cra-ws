#ifndef ROBOT_INITIALIZER_HPP
#define ROBOT_INITIALIZER_HPP

#include <array>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "modules/servo_module.hpp"
#include "modules/data_module.hpp"

class RobotInitializer
{
public:
    RobotInitializer(
        ServoModule* servo_module,
        DataModule* data_module);

    void PosRobotInit();

    void SetAcceleration(double acceleration_rpm_per_sec);

    double GetAcceleration() const;

private:
    void RampVelocity(
        double start_rpm,
        double target_rpm);

    void HoldVelocity(
        double rpm,
        std::chrono::seconds duration);

    void SetVelocity(double rpm);

    void StopVelocity();

    ServoModule* servo_module_;
    DataModule* data_module_;

    double acceleration_rpm_per_sec_{1.0};

    static constexpr int kServoCount = servo::kServoCount;
    static constexpr auto kCommandPeriod =
        std::chrono::milliseconds(50);

    static constexpr auto kVelocityHoldTime =
        std::chrono::seconds(5);
};

#endif  // ROBOT_INITIALIZER_HPP