#pragma once

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <string>

#include "kinematics/robot_kinematics.hpp"
#include "modules/servo_module.hpp"  

class DataModule {
public:
    explicit DataModule(const rclcpp::Node::SharedPtr& node);

    std::array<double, servo::kServoCount> getInitialPositions() const;

    std::string getRobotModelPath() const;
    std::string getRobotMode() const;
    std::string getBaseFrame() const;
    std::string getEndEffectorFrame() const;

    motion::RobotKinematics::IKConfig getIKConfig() const;
    double getJacobianDamping() const;

private:
    rclcpp::Node::SharedPtr node_;
};