#include "modules/data_module.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace {

void validatePositive(double value, const std::string& name) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error(
            "Invalid parameter: " + name
        );
    }
}

}  // namespace

DataModule::DataModule(const rclcpp::Node::SharedPtr& node)
: node_(node) {
    namespace fs = std::filesystem;

    if (!node_) {
        throw std::runtime_error(
            "DataModule requires a valid ROS node"
        );
    }

    package_share_path_ =
        ament_index_cpp::get_package_share_directory("cra");

    const fs::path config_path =
        fs::path(package_share_path_) / "config" / "robot.yaml";

    if (!fs::is_regular_file(config_path)) {
        throw std::runtime_error(
            "Robot configuration not found: " +
            config_path.string()
        );
    }

    config_ = YAML::LoadFile(config_path.string());

    validateConfig();

    RCLCPP_INFO(
        node_->get_logger(),
        "DataModule initialized: %s",
        config_path.string().c_str()
    );
}

std::array<double, servo::kServoCount>
DataModule::getInitialPositions() const {
    std::array<double, servo::kServoCount> positions{};
    positions.fill(0.0);

    return positions;
}

std::string DataModule::getRobotModelPath() const {
    namespace fs = std::filesystem;

    const fs::path relative_path =
        config_["robot"]["model"]["path"].as<std::string>();

    if (relative_path.empty() || relative_path.is_absolute()) {
        throw std::runtime_error(
            "Robot model path must be relative"
        );
    }

    for (const auto& part : relative_path) {
        if (part == "..") {
            throw std::runtime_error(
                "Robot model path cannot contain '..'"
            );
        }
    }

    const fs::path model_path =
        fs::path(package_share_path_) / relative_path;

    if (!fs::is_regular_file(model_path)) {
        throw std::runtime_error(
            "Robot model file not found: " + model_path.string()
        );
    }

    return model_path.string();
}

std::string DataModule::getRobotMode() const {
    return config_["robot"]["mode"].as<std::string>();
}

std::string DataModule::getBaseFrame() const {
    return config_["robot"]["frames"]["base"].as<std::string>();
}

std::string DataModule::getEndEffectorFrame() const {
    return config_["robot"]["frames"]["end_effector"]
        .as<std::string>();
}

motion::RobotKinematics::IKConfig
DataModule::getIKConfig() const {
    const YAML::Node inverse =
        config_["kinematics"]["inverse"];

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
    return config_["kinematics"]["jacobian"]["damping"]
        .as<double>();
}

DataModule::MotionConfig
DataModule::getMotionConfig() const {
    const YAML::Node motion = config_["motion"];
    const YAML::Node cartesian = motion["cartesian"];
    const YAML::Node joint = motion["joint"];
    const YAML::Node monitoring = motion["monitoring"];

    MotionConfig result;

    result.control_mode =
        motion["control_mode"].as<std::string>();

    result.target_source =
        motion["target_source"].as<std::string>();

    result.feedback_timeout =
        motion["feedback_timeout"].as<double>();

    result.cartesian.speed =
        cartesian["speed"].as<double>();

    result.cartesian.gain =
        cartesian["gain"].as<double>();

    result.cartesian.tolerance =
        cartesian["tolerance"].as<double>();

    result.joint.speed =
        joint["speed"].as<double>();

    result.joint.gain =
        joint["gain"].as<double>();

    result.joint.tolerance =
        joint["tolerance"].as<double>();

    result.joint.start_tolerance =
        joint["start_tolerance"].as<double>();

    result.joint.max_segment_delta =
        joint["max_segment_delta"].as<double>();

    result.monitoring.goal_timeout =
        monitoring["goal_timeout"].as<double>();

    result.monitoring.stall_timeout =
        monitoring["stall_timeout"].as<double>();

    result.monitoring.tracking_error_limit =
        monitoring["tracking_error_limit"].as<double>();

    result.monitoring.tracking_error_timeout =
        monitoring["tracking_error_timeout"].as<double>();

    // Keep existing MotionComponent mapping working during migration
    result.joint.goal_timeout =
        result.monitoring.goal_timeout;

    return result;
}

DataModule::TrajectoryConfig
DataModule::getTrajectoryConfig() const {
    const YAML::Node trajectory = config_["trajectory"];

    TrajectoryConfig result;

    result.enabled =
        trajectory["enabled"].as<bool>();

    result.distance_step =
        trajectory["distance_step"].as<double>();

    result.curve =
        trajectory["curve"].as<std::string>();

    result.replan_threshold =
        trajectory["replan_threshold"].as<double>();

    return result;
}

void DataModule::validateConfig() const {
    const std::string robot_mode = getRobotMode();

    if (robot_mode != "simulation" && robot_mode != "robot") {
        throw std::runtime_error(
            "Invalid robot mode: " + robot_mode
        );
    }

    if (
        getBaseFrame().empty() ||
        getEndEffectorFrame().empty()
    ) {
        throw std::runtime_error(
            "Robot frame names must not be empty"
        );
    }

    getRobotModelPath();

    const auto ik = getIKConfig();

    if (ik.max_iterations <= 0) {
        throw std::runtime_error(
            "IK max_iterations must be positive"
        );
    }

    validatePositive(
        ik.tolerance,
        "kinematics.inverse.tolerance"
    );

    validatePositive(
        ik.damping,
        "kinematics.inverse.damping"
    );

    validatePositive(
        ik.step_size,
        "kinematics.inverse.step_size"
    );

    validatePositive(
        ik.max_joint_step,
        "kinematics.inverse.max_joint_step"
    );

    validatePositive(
        getJacobianDamping(),
        "kinematics.jacobian.damping"
    );

    const MotionConfig motion = getMotionConfig();

    if (
        motion.control_mode != "cartesian" &&
        motion.control_mode != "joint_trajectory"
    ) {
        throw std::runtime_error(
            "Invalid control mode: " + motion.control_mode
        );
    }

    if (
        motion.target_source != "legacy" &&
        motion.target_source != "simulation_marker"
    ) {
        throw std::runtime_error(
            "Invalid target source: " + motion.target_source
        );
    }

    validatePositive(
        motion.feedback_timeout,
        "motion.feedback_timeout"
    );

    validatePositive(
        motion.cartesian.speed,
        "motion.cartesian.speed"
    );

    validatePositive(
        motion.cartesian.gain,
        "motion.cartesian.gain"
    );

    validatePositive(
        motion.cartesian.tolerance,
        "motion.cartesian.tolerance"
    );

    validatePositive(
        motion.joint.speed,
        "motion.joint.speed"
    );

    validatePositive(
        motion.joint.gain,
        "motion.joint.gain"
    );

    validatePositive(
        motion.joint.tolerance,
        "motion.joint.tolerance"
    );

    validatePositive(
        motion.joint.start_tolerance,
        "motion.joint.start_tolerance"
    );

    validatePositive(
        motion.joint.max_segment_delta,
        "motion.joint.max_segment_delta"
    );

    validatePositive(
        motion.monitoring.goal_timeout,
        "motion.monitoring.goal_timeout"
    );

    validatePositive(
        motion.monitoring.stall_timeout,
        "motion.monitoring.stall_timeout"
    );

    validatePositive(
        motion.monitoring.tracking_error_limit,
        "motion.monitoring.tracking_error_limit"
    );

    validatePositive(
        motion.monitoring.tracking_error_timeout,
        "motion.monitoring.tracking_error_timeout"
    );

    const TrajectoryConfig trajectory = getTrajectoryConfig();

    validatePositive(
        trajectory.distance_step,
        "trajectory.distance_step"
    );

    validatePositive(
        trajectory.replan_threshold,
        "trajectory.replan_threshold"
    );

    if (trajectory.curve.empty()) {
        throw std::runtime_error(
            "Trajectory curve must not be empty"
        );
    }
}