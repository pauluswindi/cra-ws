#include "trajectory/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace motion {

TrajectoryGenerator::TrajectoryGenerator() {
    setConfig(Config{});
}

TrajectoryGenerator::TrajectoryGenerator(const Config& config) {
    setConfig(config);
}

void TrajectoryGenerator::setConfig(const Config& config) {
    if (
        !std::isfinite(config.speed) ||
        !std::isfinite(config.distance_step) ||
        config.speed <= 0.0 ||
        config.distance_step <= 0.0
    ) {
        throw std::invalid_argument(
            "Trajectory speed and distance step must be positive"
        );
    }

    const auto curve = MotionProfile::fromString(config.curve);

    config_ = config;
    curve_ = curve;

    path_.clear();
    distances_.clear();
    total_distance_ = 0.0;
    total_time_ = 0.0;

    joint_path_.clear();
    joint_times_.clear();
    joint_total_time_ = 0.0;
}

void TrajectoryGenerator::generate(const Path& path) {
    if (path.size() < 2) {
        throw std::invalid_argument(
            "Cartesian trajectory requires at least two waypoints"
        );
    }

    Path clean_path;
    std::vector<double> distances;

    clean_path.reserve(path.size());
    distances.reserve(path.size());

    double total_distance = 0.0;

    for (const auto& position : path) {
        if (!position.allFinite()) {
            throw std::invalid_argument(
                "Invalid Cartesian waypoint"
            );
        }

        if (clean_path.empty()) {
            clean_path.push_back(position);
            distances.push_back(0.0);
            continue;
        }

        const double distance =
            (position - clean_path.back()).norm();

        if (!std::isfinite(distance)) {
            throw std::invalid_argument(
                "Invalid Cartesian segment distance"
            );
        }

        if (distance <= 1e-9) {
            continue;
        }

        total_distance += distance;
        clean_path.push_back(position);
        distances.push_back(total_distance);
    }

    if (
        clean_path.size() < 2 ||
        !std::isfinite(total_distance)
    ) {
        throw std::invalid_argument(
            "Cartesian trajectory has no valid movement"
        );
    }

    const double total_time =
        total_distance / config_.speed;

    if (!std::isfinite(total_time) || total_time <= 0.0) {
        throw std::invalid_argument(
            "Invalid Cartesian trajectory duration"
        );
    }

    path_ = std::move(clean_path);
    distances_ = std::move(distances);
    total_distance_ = total_distance;
    total_time_ = total_time;
}

TrajectoryGenerator::Sample
TrajectoryGenerator::sample(double time) const {
    if (empty() || !std::isfinite(time)) {
        throw std::invalid_argument(
            "Invalid Cartesian trajectory or time"
        );
    }

    const double t = std::clamp(time, 0.0, total_time_);
    const double s = t / total_time_;

    const double progress = MotionProfile::position(curve_, s);
    const double rate =
        MotionProfile::velocity(curve_, s) / total_time_;

    const double distance = progress * total_distance_;

    const auto it = std::upper_bound(
        distances_.begin(),
        distances_.end(),
        distance
    );

    const std::size_t index = std::clamp(
        static_cast<std::size_t>(
            std::distance(distances_.begin(), it)
        ),
        std::size_t{1},
        path_.size() - 1
    );

    const double segment_start = distances_[index - 1];
    const double segment_length =
        distances_[index] - segment_start;

    const double ratio = std::clamp(
        (distance - segment_start) / segment_length,
        0.0,
        1.0
    );

    const Position delta =
        path_[index] - path_[index - 1];

    Sample result;
    result.time = t;

    result.position =
        path_[index - 1] + ratio * delta;

    result.velocity =
        delta / segment_length * (rate * total_distance_);

    if (time <= 0.0 || time >= total_time_) {
        result.velocity.setZero();
    }

    return result;
}

std::vector<TrajectoryGenerator::Sample>
TrajectoryGenerator::generateSamples() const {
    if (empty()) {
        throw std::runtime_error(
            "Cartesian trajectory has not been generated"
        );
    }

    const double estimated_count =
        std::ceil(total_distance_ / config_.distance_step);

    if (
        !std::isfinite(estimated_count) ||
        estimated_count > 1000000.0
    ) {
        throw std::runtime_error(
            "Too many Cartesian trajectory samples"
        );
    }

    const std::size_t count = std::max(
        std::size_t{1},
        static_cast<std::size_t>(estimated_count)
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

        // Find the time corresponding to each distance sample.
        for (int iteration = 0; iteration < 40; ++iteration) {
            const double middle = 0.5 * (lower + upper);

            if (MotionProfile::position(curve_, middle) < progress) {
                lower = middle;
            } else {
                upper = middle;
            }
        }

        const double s = 0.5 * (lower + upper);
        samples.push_back(sample(s * total_time_));
    }

    samples.front() = sample(0.0);
    samples.back() = sample(total_time_);

    return samples;
}

double TrajectoryGenerator::peakVelocityFactor(
    MotionProfile::Type curve
) {
    switch (curve) {
        case MotionProfile::Type::Linear:
            return 1.0;

        case MotionProfile::Type::EaseIn:
        case MotionProfile::Type::EaseOut:
            return 2.0;

        case MotionProfile::Type::SmoothStep:
            return 1.5;

        case MotionProfile::Type::SmootherStep:
            return 1.875;
    }

    throw std::invalid_argument("Invalid trajectory curve");
}

void TrajectoryGenerator::generateJoint(
    const JointPath& path,
    double max_joint_velocity
) {
    if (
        !std::isfinite(max_joint_velocity) ||
        max_joint_velocity <= 0.0
    ) {
        throw std::invalid_argument(
            "Maximum joint velocity must be positive"
        );
    }

    if (path.size() < 2) {
        throw std::invalid_argument(
            "Joint trajectory requires at least two waypoints"
        );
    }

    JointPath clean_path;
    std::vector<double> times;

    clean_path.reserve(path.size());
    times.reserve(path.size());

    const double factor = peakVelocityFactor(curve_);
    double total_time = 0.0;

    for (const auto& position : path) {
        if (!position.allFinite()) {
            throw std::invalid_argument(
                "Invalid joint waypoint"
            );
        }

        if (clean_path.empty()) {
            clean_path.push_back(position);
            times.push_back(0.0);
            continue;
        }

        const double max_delta =
            (position - clean_path.back())
                .cwiseAbs()
                .maxCoeff();

        if (!std::isfinite(max_delta)) {
            throw std::invalid_argument(
                "Invalid joint segment"
            );
        }

        if (max_delta <= 1e-9) {
            continue;
        }

        const double duration =
            factor * max_delta / max_joint_velocity;

        if (!std::isfinite(duration) || duration <= 0.0) {
            throw std::invalid_argument(
                "Invalid joint segment duration"
            );
        }

        total_time += duration;

        if (!std::isfinite(total_time)) {
            throw std::invalid_argument(
                "Invalid joint trajectory duration"
            );
        }

        clean_path.push_back(position);
        times.push_back(total_time);
    }

    if (clean_path.size() < 2) {
        throw std::invalid_argument(
            "Joint trajectory has no valid movement"
        );
    }

    joint_path_ = std::move(clean_path);
    joint_times_ = std::move(times);
    joint_total_time_ = total_time;
}

TrajectoryGenerator::JointSample
TrajectoryGenerator::sampleJoint(double time) const {
    if (jointEmpty() || !std::isfinite(time)) {
        throw std::invalid_argument(
            "Invalid joint trajectory or time"
        );
    }

    const double t = std::clamp(
        time,
        0.0,
        joint_total_time_
    );

    JointSample result;
    result.time = t;

    if (t >= joint_total_time_) {
        result.position = joint_path_.back();
        return result;
    }

    const auto it = std::upper_bound(
        joint_times_.begin(),
        joint_times_.end(),
        t
    );

    const std::size_t index = std::clamp(
        static_cast<std::size_t>(
            std::distance(joint_times_.begin(), it)
        ),
        std::size_t{1},
        joint_path_.size() - 1
    );

    const double start_time = joint_times_[index - 1];
    const double duration =
        joint_times_[index] - start_time;

    const double s = std::clamp(
        (t - start_time) / duration,
        0.0,
        1.0
    );

    const double progress =
        MotionProfile::position(curve_, s);

    const double rate =
        MotionProfile::velocity(curve_, s) / duration;

    const JointPosition delta =
        joint_path_[index] - joint_path_[index - 1];

    result.position =
        joint_path_[index - 1] + progress * delta;

    result.velocity = rate * delta;

    if (time <= 0.0) {
        result.velocity.setZero();
    }

    return result;
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

double TrajectoryGenerator::jointTotalTime() const {
    return joint_total_time_;
}

bool TrajectoryGenerator::jointEmpty() const {
    return joint_path_.size() < 2;
}

}  // namespace motion