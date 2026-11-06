#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "vision/reconstruction/voxel_reconstruction.hpp"

namespace vision::human
{

struct BodyJoint
{
    int id = 0;
    geometry_msgs::msg::Point position;
};

struct Capsule
{
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;

    double radius = 0.0;

    std::uint32_t body_id = 0;
    int segment_id = 0;

    bool radius_valid = false;
};

struct CapsuleConfig
{
    double torso_radius = 0.13;
    double head_radius = 0.09;
    double upper_arm_radius = 0.055;
    double forearm_radius = 0.045;
    double hand_radius = 0.05;

    double torso_max_radius = 0.22;
    double head_max_radius = 0.14;
    double upper_arm_max_radius = 0.10;
    double forearm_max_radius = 0.085;
    double hand_max_radius = 0.09;

    double safety_margin = 0.015;
    double radius_percentile = 0.85;

    std::size_t min_voxels = 10;
};

struct CoverageResult
{
    std::size_t total_voxels = 0;
    std::size_t covered_voxels = 0;

    double ratio() const
    {
        if (total_voxels == 0) {
            return 0.0;
        }

        return static_cast<double>(covered_voxels) /
               static_cast<double>(total_voxels);
    }
};

class HumanCapsule
{
public:
    std::vector<Capsule> generate(
        const std::vector<BodyJoint>& joints,
        const std::vector<reconstruction::Voxel>& voxels,
        std::uint32_t body_id,
        const CapsuleConfig& config = CapsuleConfig{}) const;

    CoverageResult evaluateCoverage(
        const std::vector<Capsule>& capsules,
        const std::vector<reconstruction::Voxel>& voxels,
        double voxel_size) const;
};

}  // namespace vision::human