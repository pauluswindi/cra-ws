#pragma once

#include <cstdint>
#include <string>

#include "geometry_msgs/msg/point.hpp"

namespace vision::reconstruction
{

struct ROI3D
{
    std::uint32_t object_id = 0;
    std::string object_type;

    geometry_msgs::msg::Point min;
    geometry_msgs::msg::Point max;

    bool valid = false;
};

}  // namespace vision::reconstruction