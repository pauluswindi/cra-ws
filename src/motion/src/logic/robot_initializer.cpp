#include "logic/robot_initializer.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

RobotInitializer::RobotInitializer(
    ServoModule* servo_module,
    DataModule* data_module)
: servo_module_(servo_module),
  data_module_(data_module)
{
}

void RobotInitializer::SetAcceleration(
    double acceleration_rpm_per_sec)
{
    if (!std::isfinite(acceleration_rpm_per_sec) ||
        acceleration_rpm_per_sec <= 0.0)
    {
        RCLCPP_WARN(
            servo_module_->GetLogger(),
            "Invalid acceleration %.3f RPM/s. "
            "Acceleration remains %.3f RPM/s.",
            acceleration_rpm_per_sec,
            acceleration_rpm_per_sec_);
        return;
    }

    acceleration_rpm_per_sec_ = acceleration_rpm_per_sec;

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Velocity acceleration set to %.3f RPM/s.",
        acceleration_rpm_per_sec_);
}

double RobotInitializer::GetAcceleration() const
{
    return acceleration_rpm_per_sec_;
}

void RobotInitializer::SetVelocity(double rpm)
{
    std::array<double, kServoCount> velocity{};
    velocity.fill(rpm);

    servo_module_->SetVelocity(velocity);
}

void RobotInitializer::StopVelocity()
{
    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Setting ID 0 and ID 1 velocity to 0 RPM.");

    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
        SetVelocity(0.0);
        std::this_thread::sleep_for(kCommandPeriod);
    }
}

void RobotInitializer::RampVelocity(
    double start_rpm,
    double target_rpm)
{
    if (!rclcpp::ok()) {
        return;
    }

    if (!std::isfinite(start_rpm) ||
        !std::isfinite(target_rpm))
    {
        RCLCPP_ERROR(
            servo_module_->GetLogger(),
            "Invalid velocity value. "
            "start=%.3f target=%.3f",
            start_rpm,
            target_rpm);

        return;
    }

    if (start_rpm == target_rpm)
    {
        SetVelocity(target_rpm);
        return;
    }

    const double step =
        acceleration_rpm_per_sec_ * 0.05;

    if (step <= 0.0)
    {
        RCLCPP_ERROR(
            servo_module_->GetLogger(),
            "Invalid acceleration value: %.3f RPM/s.",
            acceleration_rpm_per_sec_);

        return;
    }

    double current_rpm = start_rpm;

    const double direction =
        target_rpm > start_rpm ? 1.0 : -1.0;

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Ramp velocity %.2f -> %.2f RPM "
        "with acceleration %.2f RPM/s.",
        start_rpm,
        target_rpm,
        acceleration_rpm_per_sec_);

    while (rclcpp::ok())
    {
        const double remaining =
            std::abs(target_rpm - current_rpm);

        if (remaining <= step)
        {
            current_rpm = target_rpm;
        }
        else
        {
            current_rpm += direction * step;
        }

        SetVelocity(current_rpm);

        if (current_rpm == target_rpm)
        {
            break;
        }

        std::this_thread::sleep_for(kCommandPeriod);
    }
}

void RobotInitializer::HoldVelocity(
    double rpm,
    std::chrono::seconds duration)
{
    if (!rclcpp::ok()) {
        return;
    }

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Holding ID 0 / ID 1 at %.2f RPM for %ld seconds.",
        rpm,
        duration.count());

    const auto start_time =
        std::chrono::steady_clock::now();

    while (
        rclcpp::ok() &&
        std::chrono::steady_clock::now() - start_time < duration)
    {
        SetVelocity(rpm);

        std::this_thread::sleep_for(kCommandPeriod);
    }
}

void RobotInitializer::PosRobotInit()
{
    if (servo_module_ == nullptr)
    {
        return;
    }

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Starting servo velocity test.");

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Test configuration:");
    
    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "  Servo ID      : 0, 1");

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "  Command period: 50 ms");

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "  Acceleration  : %.2f RPM/s",
        acceleration_rpm_per_sec_);

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "  Hold time     : %ld seconds",
        kVelocityHoldTime.count());

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Disabling torque for ID 0 and ID 1.");

    servo_module_->SetTorque(0, false);
    servo_module_->SetTorque(1, false);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(200));

    if (!rclcpp::ok()) {
        return;
    }

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Switching ID 0 and ID 1 to Velocity Mode.");

    servo_module_->SetOperatingMode(0, kModeVelocity);
    servo_module_->SetOperatingMode(1, kModeVelocity);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(200));

    if (!rclcpp::ok()) {
        return;
    }

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Enabling torque for ID 0 and ID 1.");

    servo_module_->SetTorque(0, true);
    servo_module_->SetTorque(1, true);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(200));

    if (!rclcpp::ok()) {
        return;
    }

    StopVelocity();

    if (!rclcpp::ok()) {
        return;
    }

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Starting positive velocity test.");

    RampVelocity(0.0, 1.0);
    HoldVelocity(1.0, kVelocityHoldTime);

    RampVelocity(1.0, 2.0);
    HoldVelocity(2.0, kVelocityHoldTime);

    RampVelocity(2.0, 3.0);
    HoldVelocity(3.0, kVelocityHoldTime);

    if (!rclcpp::ok()) {
        StopVelocity();
        return;
    }

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Returning from +3 RPM to 0 RPM.");

    RampVelocity(3.0, 2.0);
    HoldVelocity(2.0, kVelocityHoldTime);

    RampVelocity(2.0, 1.0);
    HoldVelocity(1.0, kVelocityHoldTime);

    RampVelocity(1.0, 0.0);

    StopVelocity();

    if (!rclcpp::ok()) {
        return;
    }

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Positive velocity test completed.");

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Starting negative velocity test.");

    RampVelocity(0.0, -1.0);
    HoldVelocity(-1.0, kVelocityHoldTime);

    RampVelocity(-1.0, -2.0);
    HoldVelocity(-2.0, kVelocityHoldTime);

    RampVelocity(-2.0, -3.0);
    HoldVelocity(-3.0, kVelocityHoldTime);

    if (!rclcpp::ok()) {
        StopVelocity();
        return;
    }

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Returning from -3 RPM to 0 RPM.");

    RampVelocity(-3.0, -2.0);
    HoldVelocity(-2.0, kVelocityHoldTime);

    RampVelocity(-2.0, -1.0);
    HoldVelocity(-1.0, kVelocityHoldTime);

    RampVelocity(-1.0, 0.0);

    StopVelocity();

    if (!rclcpp::ok()) {
        return;
    }

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Servo velocity test completed.");

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Final velocity: ID 0 = 0 RPM, ID 1 = 0 RPM.");

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Disabling torque for ID 0 and ID 1.");

    servo_module_->SetTorque(0, false);
    servo_module_->SetTorque(1, false);

    RCLCPP_INFO(
        servo_module_->GetLogger(),
        "Servo test finished successfully.");
}