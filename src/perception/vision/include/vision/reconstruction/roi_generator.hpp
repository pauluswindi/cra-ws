#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

#include "vision/reconstruction/roi.hpp"

namespace vision::reconstruction
{

class ROIGenerator
{
public:
    ROI3D generate(
        const std::vector<geometry_msgs::msg::Point>& points,
        double margin,
        std::uint32_t object_id,
        const std::string& object_type) const;
};

}  // namespace vision::reconstruction