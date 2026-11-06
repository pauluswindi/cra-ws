#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "vision/reconstruction/voxel_reconstruction.hpp"

namespace vision::reconstruction
{

inline void appendVoxelMarker(
    const std::vector<Voxel>& voxels,
    std::uint32_t body_id,
    double voxel_size,
    const builtin_interfaces::msg::Time& stamp,
    visualization_msgs::msg::MarkerArray& output)
{
    using Marker = visualization_msgs::msg::Marker;

    if (voxels.empty() ||
        !std::isfinite(voxel_size) ||
        voxel_size <= 0.0) {
        return;
    }

    Marker marker;

    marker.header.frame_id = "base_link";
    marker.header.stamp = stamp;

    marker.ns = "human_voxels";
    marker.id = static_cast<int>(body_id);

    marker.type = Marker::CUBE_LIST;
    marker.action = Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = voxel_size;
    marker.scale.y = voxel_size;
    marker.scale.z = voxel_size;

    marker.color.r = 0.1f;
    marker.color.g = 0.8f;
    marker.color.b = 1.0f;
    marker.color.a = 0.9f;

    // Keep the marker until explicitly removed.
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 0;

    marker.points.reserve(voxels.size());

    for (const auto& voxel : voxels) {
        if (!std::isfinite(voxel.center.x) ||
            !std::isfinite(voxel.center.y) ||
            !std::isfinite(voxel.center.z)) {
            continue;
        }

        marker.points.push_back(voxel.center);
    }

    if (!marker.points.empty()) {
        output.markers.push_back(std::move(marker));
    }
}

}  // namespace vision::reconstruction