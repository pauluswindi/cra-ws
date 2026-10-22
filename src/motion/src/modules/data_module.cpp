#include "modules/data_module.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace {

YAML::Node loadRobotConfig() {
    namespace fs = std::filesystem;

    const fs::path config_path =
        fs::path(ament_index_cpp::get_package_share_directory("cra")) /
        "config" / "robot.yaml";

    return YAML::LoadFile(config_path.string());
}

}  // namespace

DataModule::DataModule(const rclcpp::Node::SharedPtr& node)
: node_(node) {
    RCLCPP_INFO(node_->get_logger(), "DataModule initialized.");
}

std::array<double, servo::kServoCount>
DataModule::getInitialPositions() const {
    std::array<double, servo::kServoCount> positions;
    positions.fill(0.0);

    return positions;
}

std::string DataModule::getRobotModelPath() const {
    namespace fs = std::filesystem;

    const YAML::Node config = loadRobotConfig();

    const fs::path cra_share =
        ament_index_cpp::get_package_share_directory("cra");

    const fs::path relative_path =
        config["robot"]["model"]["path"].as<std::string>();

    if (relative_path.is_absolute()) {
        throw std::runtime_error(
            "Robot model path must be relative to the cra package"
        );
    }

    const fs::path model_path = cra_share / relative_path;

    if (!fs::is_regular_file(model_path)) {
        throw std::runtime_error(
            "Robot XML not found: " + model_path.string()
        );
    }

    return model_path.string();
}

std::string DataModule::getRobotMode() const {
    const YAML::Node config = loadRobotConfig();

    return config["robot"]["mode"].as<std::string>();
}

std::string DataModule::getBaseFrame() const {
    const YAML::Node config = loadRobotConfig();

    return config["robot"]["frames"]["base"].as<std::string>();
}

std::string DataModule::getEndEffectorFrame() const {
    const YAML::Node config = loadRobotConfig();

    return config["robot"]["frames"]["end_effector"].as<std::string>();
}

motion::RobotKinematics::IKConfig
DataModule::getIKConfig() const {
    const YAML::Node config = loadRobotConfig();
    const YAML::Node inverse = config["kinematics"]["inverse"];

    motion::RobotKinematics::IKConfig result;

    result.max_iterations =
        inverse["max_iterations"].as<int>();

    result.tolerance =
        inverse["tolerance"].as<double>();

    result.damping =
        inverse["damping"].as<double>();

    result.step_size =
        inverse["step_size"].as<double>();

    result.max_joint_step =
        inverse["max_joint_step"].as<double>();

    return result;
}

double DataModule::getJacobianDamping() const {
    const YAML::Node config = loadRobotConfig();

    const double damping =
        config["kinematics"]["jacobian"]["damping"].as<double>();

    if (!std::isfinite(damping) || damping <= 0.0) {
        throw std::runtime_error(
            "Jacobian damping must be positive and finite"
        );
    }

    return damping;
}