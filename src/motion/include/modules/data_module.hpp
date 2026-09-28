#ifndef MODULES_DATA_MODULE_HPP_
#define MODULES_DATA_MODULE_HPP_

#include "common.hpp"
#include <array>
#include "rclcpp/rclcpp.hpp"

class DataModule {
public:
    explicit DataModule(const rclcpp::Node::SharedPtr& node);
    
    std::array<double, servo::kServoCount> getInitialPositions() const;

    rclcpp::Logger GetLogger() const { return node_->get_logger(); }

private:
    rclcpp::Node::SharedPtr node_;
};

#endif // MODULES_DATA_MODULE_HPP_