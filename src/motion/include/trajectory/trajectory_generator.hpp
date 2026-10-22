#ifndef TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Core>

#include <string>
#include <vector>

#include "trajectory/motion_profile.hpp"

namespace motion {

class TrajectoryGenerator {
public:
    using Position = Eigen::Vector3d;
    using Path = std::vector<Position>;

    struct Sample {
        Position position = Position::Zero();
        Position velocity = Position::Zero();
        double time = 0.0;
    };

    struct Config {
        double speed = 0.1;
        double distance_step = 0.01;
        std::string curve = "smoother_step";
    };

    TrajectoryGenerator();
    explicit TrajectoryGenerator(const Config& config);

    void setConfig(const Config& config);

    void generate(const Path& path);

    Sample sample(double time) const;

    std::vector<Sample> generateSamples() const;

    double totalDistance() const;
    double totalTime() const;

    bool empty() const;

private:
    Position positionAtDistance(double distance) const;
    Position directionAtDistance(double distance) const;

    Config config_;

    MotionProfile::Type profile_type_ =
        MotionProfile::Type::SmootherStep;

    Path path_;
    std::vector<double> cumulative_distances_;

    double total_distance_{0.0};
    double total_time_{0.0};
};

}  // namespace motion

#endif  // TRAJECTORY_GENERATOR_HPP_