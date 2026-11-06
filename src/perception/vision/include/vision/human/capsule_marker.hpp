#pragma once

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "vision/human/human_capsule.hpp"

namespace vision::human
{

using MarkerId = std::pair<std::string, int>;
using MarkerIds = std::set<MarkerId>;

inline void appendCapsuleMarkers(
    const std::vector<Capsule>& capsules,
    const builtin_interfaces::msg::Time& stamp,
    visualization_msgs::msg::MarkerArray& output,
    MarkerIds& current_ids)
{
    using Marker = visualization_msgs::msg::Marker;

    for (const auto& capsule : capsules) {
        const auto& a = capsule.start;
        const auto& b = capsule.end;

        if (!std::isfinite(capsule.radius) ||
            capsule.radius <= 0.0) {
            continue;
        }

        const tf2::Vector3 start(a.x, a.y, a.z);
        const tf2::Vector3 end(b.x, b.y, b.z);
        const tf2::Vector3 direction = end - start;

        const double length = direction.length();
        const double diameter = 2.0 * capsule.radius;

        if (!std::isfinite(length)) {
            continue;
        }

        Marker marker;

        marker.header.frame_id = "base_link";
        marker.header.stamp = stamp;

        marker.ns =
            "human_capsule_" + std::to_string(capsule.body_id);

        marker.action = Marker::ADD;

        marker.color.r = 1.0f;
        marker.color.g = 0.55f;
        marker.color.b = 0.1f;
        marker.color.a = 0.25f;

        // Markers persist until explicitly deleted.
        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 0;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = diameter;
        marker.scale.y = diameter;
        marker.scale.z = diameter;

        const int base_id = capsule.segment_id * 3;

        if (length < 1e-6) {
            marker.id = base_id;
            marker.type = Marker::SPHERE;
            marker.pose.position = a;

            current_ids.emplace(marker.ns, marker.id);
            output.markers.push_back(marker);
            continue;
        }

        // Align the cylinder's Z axis with the body segment.
        const tf2::Vector3 axis = direction / length;
        const tf2::Vector3 z_axis(0.0, 0.0, 1.0);

        tf2::Quaternion rotation;
        rotation.setRotation(
            z_axis.cross(axis).length2() > 1e-12
                ? z_axis.cross(axis).normalized()
                : tf2::Vector3(1.0, 0.0, 0.0),
            std::acos(std::clamp(axis.z(), -1.0, 1.0)));

        rotation.normalize();

        marker.id = base_id;
        marker.type = Marker::CYLINDER;

        marker.pose.position.x = (a.x + b.x) * 0.5;
        marker.pose.position.y = (a.y + b.y) * 0.5;
        marker.pose.position.z = (a.z + b.z) * 0.5;

        marker.pose.orientation.x = rotation.x();
        marker.pose.orientation.y = rotation.y();
        marker.pose.orientation.z = rotation.z();
        marker.pose.orientation.w = rotation.w();

        marker.scale.z = length;

        current_ids.emplace(marker.ns, marker.id);
        output.markers.push_back(marker);

        marker.type = Marker::SPHERE;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = diameter;

        marker.id = base_id + 1;
        marker.pose.position = a;

        current_ids.emplace(marker.ns, marker.id);
        output.markers.push_back(marker);

        marker.id = base_id + 2;
        marker.pose.position = b;

        current_ids.emplace(marker.ns, marker.id);
        output.markers.push_back(marker);
    }
}

}  // namespace vision::human