#pragma once

#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "msgs/msg/human_capsule.hpp"
#include "msgs/msg/human_capsule_array.hpp"

#include "vision/human/human_capsule.hpp"

namespace vision::human
{

inline msgs::msg::HumanCapsuleArray makeGeometryMessage(
    const std::vector<Capsule>& capsules,
    const builtin_interfaces::msg::Time& stamp)
{
    msgs::msg::HumanCapsuleArray output;

    output.header.frame_id = "base_link";
    output.header.stamp = stamp;
    output.valid = true;

    output.capsules.reserve(capsules.size());

    for (const auto& capsule : capsules) {
        msgs::msg::HumanCapsule geometry;

        geometry.body_id = capsule.body_id;
        geometry.segment_id = capsule.segment_id;
        geometry.start = capsule.start;
        geometry.end = capsule.end;
        geometry.radius = capsule.radius;

        output.capsules.push_back(geometry);
    }

    return output;
}

}  // namespace vision::human