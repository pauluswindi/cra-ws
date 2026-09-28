#include "modules/data_module.hpp"

DataModule::DataModule(const rclcpp::Node::SharedPtr& node)
: node_(node) {
    RCLCPP_INFO(node_->get_logger(), "DataModule initialized for testing.");
}

std::array<double, servo::kServoCount> DataModule::getInitialPositions() const {
    std::array<double, servo::kServoCount> positions;
    positions.fill(0.0);
    
    return positions;
}