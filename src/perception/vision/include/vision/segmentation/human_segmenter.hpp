#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sensor_msgs/msg/image.hpp"
#include "vision/segmentation/pixel_run.hpp"

namespace vision::segmentation
{

class HumanSegmenter
{
public:
    std::vector<std::uint8_t> generateMask(
        const sensor_msgs::msg::Image& body_index_map,
        std::uint32_t body_id) const;

    // Ordered runs of every matching pixel; no image subsampling.
    std::vector<PixelRun> generatePixelRuns(
        const sensor_msgs::msg::Image& body_index_map,
        std::uint32_t body_id) const;
};

}  // namespace vision::segmentation
