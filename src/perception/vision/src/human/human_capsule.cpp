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

    const double closest_x = start.x + t * dx;
    const double closest_y = start.y + t * dy;
    const double closest_z = start.z + t * dz;

    const double px = point.x - closest_x;
    const double py = point.y - closest_y;
    const double pz = point.z - closest_z;

    return std::sqrt(px * px + py * py + pz * pz);
}

void validateConfig(const CapsuleConfig& config)
{
    const std::array<double, 4> default_radii = {{
        config.torso_radius,
        config.head_radius,
        config.upper_arm_radius,
        config.forearm_radius
    }};

    const std::array<double, 4> max_radii = {{
        config.torso_max_radius,
        config.head_max_radius,
        config.upper_arm_max_radius,
        config.forearm_max_radius
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
            max_radii[i] < default_radii[i] +
                config.safety_margin) {
            throw std::invalid_argument(
                "Invalid human capsule radius limits");
        }
    }
}

double estimateRadius(
    std::vector<double>& distances,
    double default_radius,
    double max_radius,
    const CapsuleConfig& config)
{
    double estimated_radius = default_radius;

    if (distances.size() >= config.min_voxels) {
        const std::size_t index = static_cast<std::size_t>(
            config.radius_percentile *
            static_cast<double>(distances.size() - 1));

        std::nth_element(
            distances.begin(),
            distances.begin() + index,
            distances.end());

        estimated_radius = std::max(
            default_radius,
            distances[index]);
    }

    return std::min(
        max_radius,
        estimated_radius + config.safety_margin);
}

}  // namespace

std::vector<Capsule> HumanCapsule::generate(
    const std::vector<BodyJoint>& joints,
    const std::vector<reconstruction::Voxel>& voxels,
    std::uint32_t body_id,
    const CapsuleConfig& config) const
{
    validateConfig(config);

    const std::array<SegmentDefinition, 6> segments = {{
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
            config.forearm_max_radius}
    }};

    std::vector<Capsule> capsules;
    capsules.reserve(segments.size());

    std::vector<double> max_radii;
    max_radii.reserve(segments.size());

    // Generate the initial skeleton-based capsules.
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
        capsule.radius = segment.default_radius;
        capsule.body_id = body_id;
        capsule.segment_id = segment.id;

        capsules.push_back(capsule);
        max_radii.push_back(segment.max_radius);
    }

    if (capsules.empty()) {
        return capsules;
    }

    // Store voxel distances separately for each capsule.
    std::vector<std::vector<double>> segment_distances(
        capsules.size());

    // Assign each voxel to at most one nearby body segment.
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

            double projection = 0.0;

            const double distance = distanceToSegment(
                point,
                capsule.start,
                capsule.end,
                projection);

            if (!std::isfinite(distance) ||
                distance > max_radii[i]) {
                continue;
            }

            const bool is_sphere =
                squaredDistance(
                    capsule.start,
                    capsule.end) < 1e-12;

            // Use the central segment to avoid joint overlap.
            if (!is_sphere &&
                (projection < 0.15 || projection > 0.85)) {
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

    // Refine each capsule radius using its assigned voxels.
    for (std::size_t i = 0; i < capsules.size(); ++i) {
        const auto& segment = segments;

        (void)segment;

        const auto definition = std::find_if(
            segments.begin(),
            segments.end(),
            [&capsules, i](const SegmentDefinition& candidate) {
                return candidate.id == capsules[i].segment_id;
            });

        if (definition == segments.end()) {
            continue;
        }

        capsules[i].radius = estimateRadius(
            segment_distances[i],
            definition->default_radius,
            definition->max_radius,
            config);
    }

    return capsules;
}

}  // namespace vision::human