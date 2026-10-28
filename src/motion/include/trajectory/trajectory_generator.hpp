#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

#include "trajectory/motion_profile.hpp"

namespace motion {

class TrajectoryGenerator {
public:
    using Position = Eigen::Vector3d;
    using Path = std::vector<Position>;

    using JointPosition = Eigen::Matrix<double, 5, 1>;
    using JointPath = std::vector<JointPosition>;

    struct Config {
        double speed{0.1};
        double distance_step{0.01};
        std::string curve{"linear"};
    };

    struct Sample {
        Position position = Position::Zero();
        Position velocity = Position::Zero();
        double time{0.0};
    };

    struct JointSample {
        JointPosition position = JointPosition::Zero();
        JointPosition velocity = JointPosition::Zero();
        double time{0.0};
    };

    TrajectoryGenerator();
    explicit TrajectoryGenerator(const Config& config);

    void setConfig(const Config& config);

    // Cartesian trajectory
    void generate(const Path& path);
    Sample sample(double time) const;
    std::vector<Sample> generateSamples() const;

    double totalDistance() const;
    double totalTime() const;
    bool empty() const;

    // Synchronized joint trajectory
    void generateJoint(
        const JointPath& path,
        double max_joint_velocity
    );

    JointSample sampleJoint(double time) const;

    double jointTotalTime() const;
    bool jointEmpty() const;

private:
    static double peakVelocityFactor(MotionProfile::Type curve);

    Config config_;
    MotionProfile::Type curve_{MotionProfile::Type::Linear};

    Path path_;
    std::vector<double> distances_;
    double total_distance_{0.0};
    double total_time_{0.0};

    JointPath joint_path_;
    std::vector<double> joint_times_;
    double joint_total_time_{0.0};
};

}  // namespace motion