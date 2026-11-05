#pragma once

#include <cstdint>
#include <vector>

#include "sensor_msgs/msg/image.hpp"

namespace vision::segmentation
{

class HumanSegmenter
{
public:
    std::vector<std::uint8_t> generateMask(
        const sensor_msgs::msg::Image& body_index_map,
        std::uint32_t body_id) const;
};

}  // namespace vision::segmentation