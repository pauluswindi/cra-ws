#include "trajectory/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace motion {

TrajectoryGenerator::TrajectoryGenerator() {
    setConfig(Config{});
}

TrajectoryGenerator::TrajectoryGenerator(
    const Config& config
) {
    setConfig(config);
}

void TrajectoryGenerator::setConfig(
    const Config& config
) {
    if (
        !std::isfinite(config.speed) ||
        !std::isfinite(config.distance_step) ||
        config.speed <= 0.0 ||
        config.distance_step <= 0.0
    ) {
        throw std::invalid_argument(
            "Trajectory speed and distance_step must be positive"
        );
    }

    const auto profile_type =
        MotionProfile::fromString(config.curve);

    config_ = config;
    profile_type_ = profile_type;

    path_.clear();
    cumulative_distances_.clear();

    total_distance_ = 0.0;
    total_time_ = 0.0;
}

void TrajectoryGenerator::generate(
    const Path& path
) {
    if (path.size() < 2) {
        throw std::invalid_argument(
            "Trajectory requires at least two waypoints"
        );
    }

    Path clean_path;
    clean_path.reserve(path.size());

    for (const auto& position : path) {
        if (!position.allFinite()) {
            throw std::invalid_argument(
                "Trajectory contains non-finite positions"
            );
        }

        if (
            clean_path.empty() ||
            (position - clean_path.back()).norm() > 1e-9
        ) {
            clean_path.push_back(position);
        }
    }

    if (clean_path.size() < 2) {
        throw std::invalid_argument(
            "Trajectory has no movement"
        );
    }

    std::vector<double> distances;
    distances.reserve(clean_path.size());
    distances.push_back(0.0);

    double total_distance = 0.0;

    for (std::size_t i = 1; i < clean_path.size(); ++i) {
        total_distance += (
            clean_path[i] - clean_path[i - 1]
        ).norm();

        distances.push_back(total_distance);
    }

    if (
        !std::isfinite(total_distance) ||
        total_distance <= 1e-9
    ) {
        throw std::invalid_argument(
            "Invalid trajectory distance"
        );
    }

    const double total_time =
        total_distance / config_.speed;

    if (
        !std::isfinite(total_time) ||
        total_time <= 0.0
    ) {
        throw std::invalid_argument(
            "Invalid trajectory duration"
        );
    }

    path_ = std::move(clean_path);
    cumulative_distances_ = std::move(distances);

    total_distance_ = total_distance;
    total_time_ = total_time;
}

TrajectoryGenerator::Position
TrajectoryGenerator::positionAtDistance(
    double distance
) const {
    if (distance <= 0.0) {
        return path_.front();
    }

    if (distance >= total_distance_) {
        return path_.back();
    }

    const auto it = std::upper_bound(
        cumulative_distances_.begin(),
        cumulative_distances_.end(),
        distance
    );

    const std::size_t index = static_cast<std::size_t>(
        std::distance(cumulative_distances_.begin(), it)
    );

    const double start_distance =
        cumulative_distances_[index - 1];

    const double segment_distance =
        cumulative_distances_[index] - start_distance;

    const double ratio =
        (distance - start_distance) / segment_distance;

    return path_[index - 1] +
        ratio * (path_[index] - path_[index - 1]);
}

TrajectoryGenerator::Position
TrajectoryGenerator::directionAtDistance(
    double distance
) const {
    const double clamped_distance = std::clamp(
        distance,
        0.0,
        total_distance_
    );

    const auto it = std::upper_bound(
        cumulative_distances_.begin(),
        cumulative_distances_.end(),
        clamped_distance
    );

    const std::size_t index = std::min(
        static_cast<std::size_t>(
            std::distance(cumulative_distances_.begin(), it)
        ),
        path_.size() - 1
    );

    return (
        path_[index] - path_[index - 1]
    ).normalized();
}

TrajectoryGenerator::Sample
TrajectoryGenerator::sample(
    double time
) const {
    if (empty()) {
        throw std::runtime_error(
            "Trajectory has not been generated"
        );
    }

    if (!std::isfinite(time)) {
        throw std::invalid_argument(
            "Trajectory time must be finite"
        );
    }

    const double clamped_time = std::clamp(
        time,
        0.0,
        total_time_
    );

    const double normalized_time =
        clamped_time / total_time_;

    const double progress = MotionProfile::position(
        profile_type_,
        normalized_time
    );

    const double progress_rate =
        MotionProfile::velocity(
            profile_type_,
            normalized_time
        ) / total_time_;

    const double distance =
        progress * total_distance_;

    Sample result;

    result.time = clamped_time;
    result.position = positionAtDistance(distance);

    result.velocity = directionAtDistance(distance) *
        (progress_rate * total_distance_);

    if (time <= 0.0 || time >= total_time_) {
        result.velocity.setZero();
    }

    return result;
}

std::vector<TrajectoryGenerator::Sample>
TrajectoryGenerator::generateSamples() const {
    if (empty()) {
        throw std::runtime_error(
            "Trajectory has not been generated"
        );
    }

    const double estimated_count =
        std::ceil(total_distance_ / config_.distance_step);

    if (
        !std::isfinite(estimated_count) ||
        estimated_count > 1000000.0
    ) {
        throw std::runtime_error(
            "Trajectory requires too many samples"
        );
    }

    const std::size_t count = static_cast<std::size_t>(
        estimated_count
    );

    std::vector<Sample> samples;
    samples.reserve(count + 1);

    for (std::size_t i = 0; i <= count; ++i) {
        const double distance = std::min(
            static_cast<double>(i) * config_.distance_step,
            total_distance_
        );

        const double progress =
            distance / total_distance_;

        double lower = 0.0;
        double upper = 1.0;

        // Invert the monotonic motion profile to find sample time.
        for (int iteration = 0; iteration < 50; ++iteration) {
            const double middle =
                0.5 * (lower + upper);

            if (
                MotionProfile::position(
                    profile_type_,
                    middle
                ) < progress
            ) {
                lower = middle;
            } else {
                upper = middle;
            }
        }

        const double normalized_time =
            0.5 * (lower + upper);

        samples.push_back(
            sample(normalized_time * total_time_)
        );
    }

    samples.front() = sample(0.0);
    samples.back() = sample(total_time_);

    return samples;
}

double TrajectoryGenerator::totalDistance() const {
    return total_distance_;
}

double TrajectoryGenerator::totalTime() const {
    return total_time_;
}

bool TrajectoryGenerator::empty() const {
    return path_.size() < 2;
}

}  // namespace motion