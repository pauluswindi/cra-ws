#include "vision/human/human_capsule.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace vision::human
{

namespace
{

struct SegmentDefinition
{
    int id;
    int start_joint;
    int end_joint;

    double default_radius;
    double max_radius;
};

bool isFinite(const geometry_msgs::msg::Point& point)
{
    return std::isfinite(point.x) &&
           std::isfinite(point.y) &&
           std::isfinite(point.z);
}

const geometry_msgs::msg::Point* findJoint(
    const std::vector<BodyJoint>& joints,
    int joint_id)
{
    for (const auto& joint : joints) {
        if (joint.id == joint_id && isFinite(joint.position)) {
            return &joint.position;
        }
    }

    return nullptr;
}

double squaredDistance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;

    return dx * dx + dy * dy + dz * dz;
}

double distanceToSegment(
    const geometry_msgs::msg::Point& point,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& end,
    double& projection)
{
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double dz = end.z - start.z;

    const double length_squared =
        dx * dx + dy * dy + dz * dz;

    if (length_squared < 1e-12) {
        projection = 0.0;
        return std::sqrt(squaredDistance(point, start));
    }

    projection =
        ((point.x - start.x) * dx +
         (point.y - start.y) * dy +
         (point.z - start.z) * dz) /
        length_squared;

    const double t = std::clamp(projection, 0.0, 1.0);

    const double px = point.x - (start.x + t * dx);
    const double py = point.y - (start.y + t * dy);
    const double pz = point.z - (start.z + t * dz);

    return std::sqrt(px * px + py * py + pz * pz);
}

void validateConfig(const CapsuleConfig& config)
{
    const std::array<double, 5> default_radii = {{
        config.torso_radius,
        config.head_radius,
        config.upper_arm_radius,
        config.forearm_radius,
        config.hand_radius
    }};

    const std::array<double, 5> max_radii = {{
        config.torso_max_radius,
        config.head_max_radius,
        config.upper_arm_max_radius,
        config.forearm_max_radius,
        config.hand_max_radius
    }};

    if (!std::isfinite(config.safety_margin) ||
        config.safety_margin < 0.0 ||
        !std::isfinite(config.radius_percentile) ||
        config.radius_percentile <= 0.0 ||
        config.radius_percentile > 1.0 ||
        config.min_voxels == 0) {
        throw std::invalid_argument(
            "Invalid human capsule configuration");
    }

    for (std::size_t i = 0; i < default_radii.size(); ++i) {
        if (!std::isfinite(default_radii[i]) ||
            !std::isfinite(max_radii[i]) ||
            default_radii[i] <= 0.0 ||
            max_radii[i] <
                default_radii[i] + config.safety_margin) {
            throw std::invalid_argument(
                "Invalid human capsule radius limits");
        }
    }
}

double estimateRadius(
    std::vector<double>& distances,
    const SegmentDefinition& segment,
    const CapsuleConfig& config)
{
    double radius = segment.default_radius;

    if (distances.size() >= config.min_voxels) {
        // Interpolate between neighboring ranks instead of snapping to
        // one voxel distance when the number of samples changes.
        const double rank = config.radius_percentile *
            static_cast<double>(distances.size() - 1);
        const auto lower_index = static_cast<std::size_t>(std::floor(rank));
        const auto upper_index = static_cast<std::size_t>(std::ceil(rank));

        std::nth_element(
            distances.begin(),
            distances.begin() + lower_index,
            distances.end());
        const double lower_value = distances[lower_index];

        double estimated = lower_value;
        if (upper_index != lower_index) {
            std::nth_element(
                distances.begin(),
                distances.begin() + upper_index,
                distances.end());
            const double upper_value = distances[upper_index];
            estimated += (rank - static_cast<double>(lower_index)) *
                (upper_value - lower_value);
        }

        radius = std::max(segment.default_radius, estimated);
    }

    // Keep the configurable margin inside the configured maximum radius.
    return std::clamp(
        radius + config.safety_margin,
        segment.default_radius + config.safety_margin,
        segment.max_radius);
}

}  // namespace

std::vector<Capsule> HumanCapsule::generate(
    const std::vector<BodyJoint>& joints,
    const std::vector<reconstruction::Voxel>& voxels,
    std::uint32_t body_id,
    const CapsuleConfig& config) const
{
    validateConfig(config);

    const std::array<SegmentDefinition, 8> segments = {{
        {0, 0, 2,
            config.torso_radius,
            config.torso_max_radius},

        {1, 26, 26,
            config.head_radius,
            config.head_max_radius},

        {2, 5, 6,
            config.upper_arm_radius,
            config.upper_arm_max_radius},

        {3, 6, 7,
            config.forearm_radius,
            config.forearm_max_radius},

        {4, 12, 13,
            config.upper_arm_radius,
            config.upper_arm_max_radius},

        {5, 13, 14,
            config.forearm_radius,
            config.forearm_max_radius},

        {6, 7, 9,
            config.hand_radius,
            config.hand_max_radius},

        {7, 14, 16,
            config.hand_radius,
            config.hand_max_radius}
    }};

    std::vector<Capsule> capsules;
    std::vector<SegmentDefinition> active_segments;

    capsules.reserve(segments.size());
    active_segments.reserve(segments.size());

    // Generate capsules from valid skeleton joints.
    for (const auto& segment : segments) {
        const auto* start = findJoint(
            joints,
            segment.start_joint);

        const auto* end = findJoint(
            joints,
            segment.end_joint);

        if (start == nullptr || end == nullptr) {
            continue;
        }

        if (segment.start_joint != segment.end_joint &&
            squaredDistance(*start, *end) < 1e-8) {
            continue;
        }

        Capsule capsule;

        capsule.start = *start;
        capsule.end = *end;

        capsule.radius =
            segment.default_radius + config.safety_margin;

        capsule.body_id = body_id;
        capsule.segment_id = segment.id;
        capsule.radius_valid = false;

        capsules.push_back(capsule);
        active_segments.push_back(segment);
    }

    if (capsules.empty()) {
        return capsules;
    }

    std::vector<std::vector<double>> segment_distances(
        capsules.size());

    // Assign each voxel to its nearest eligible segment.
    for (const auto& voxel : voxels) {
        const auto& point = voxel.center;

        if (!isFinite(point)) {
            continue;
        }

        std::size_t nearest_segment = capsules.size();

        double nearest_distance =
            std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < capsules.size(); ++i) {
            const auto& capsule = capsules[i];
            const auto& segment = active_segments[i];

            double projection = 0.0;

            const double distance = distanceToSegment(
                point,
                capsule.start,
                capsule.end,
                projection);

            if (!std::isfinite(distance) ||
                distance > segment.max_radius) {
                continue;
            }

            const bool is_sphere =
                segment.start_joint == segment.end_joint;

            const bool is_hand =
                segment.id == 6 || segment.id == 7;

            // Use the central portion for arm and torso fitting.
            if (!is_sphere && !is_hand &&
                (projection < 0.15 || projection > 0.85)) {
                continue;
            }

            // Hand fitting uses the entire wrist-to-fingertip axis.
            if (is_hand &&
                (projection < 0.0 || projection > 1.0)) {
                continue;
            }

            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_segment = i;
            }
        }

        if (nearest_segment < capsules.size()) {
            segment_distances[nearest_segment].push_back(
                nearest_distance);
        }
    }

    // Estimate radius from assigned voxel distances.
    for (std::size_t i = 0; i < capsules.size(); ++i) {
        auto& distances = segment_distances[i];

        capsules[i].radius_valid =
            distances.size() >= config.min_voxels;

        capsules[i].radius = estimateRadius(
            distances,
            active_segments[i],
            config);
    }

    return capsules;
}

CoverageResult HumanCapsule::evaluateCoverage(
    const std::vector<Capsule>& capsules,
    const std::vector<reconstruction::Voxel>& voxels,
    double voxel_size) const
{
    if (!std::isfinite(voxel_size) || voxel_size <= 0.0) {
        throw std::invalid_argument(
            "Invalid voxel size for coverage evaluation");
    }

    CoverageResult result;

    // Account for the distance from voxel center to its corners.
    const double voxel_tolerance =
        0.5 * std::sqrt(3.0) * voxel_size;

    for (const auto& voxel : voxels) {
        if (!isFinite(voxel.center)) {
            continue;
        }

        ++result.total_voxels;

        for (const auto& capsule : capsules) {
            if (!isFinite(capsule.start) ||
                !isFinite(capsule.end) ||
                !std::isfinite(capsule.radius) ||
                capsule.radius <= 0.0) {
                continue;
            }

            double projection = 0.0;

            const double distance = distanceToSegment(
                voxel.center,
                capsule.start,
                capsule.end,
                projection);

            if (distance <=
                capsule.radius + voxel_tolerance) {
                ++result.covered_voxels;
                break;
            }
        }
    }

    return result;
}

}  // namespace vision::human