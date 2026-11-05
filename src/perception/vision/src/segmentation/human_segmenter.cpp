#include "vision/segmentation/human_segmenter.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "sensor_msgs/image_encodings.hpp"

namespace vision::segmentation
{

std::vector<std::uint8_t> HumanSegmenter::generateMask(
    const sensor_msgs::msg::Image& body_index_map,
    std::uint32_t body_id) const
{
    if (body_index_map.encoding !=
        sensor_msgs::image_encodings::MONO8) {
        throw std::invalid_argument(
            "Body Index Map must use mono8 encoding");
    }

    const std::size_t width = body_index_map.width;
    const std::size_t height = body_index_map.height;
    const std::size_t step = body_index_map.step;

    if (width == 0 || height == 0 || step < width) {
        throw std::invalid_argument(
            "Invalid Body Index Map dimensions");
    }

    if (body_index_map.data.size() / step < height) {
        throw std::invalid_argument(
            "Invalid Body Index Map buffer");
    }

    constexpr std::uint8_t kBackgroundIndex = 255;

    // Match the body ID encoding used by the CRA Azure Kinect driver.
    const auto target_index =
        static_cast<std::uint8_t>(body_id % kBackgroundIndex);

    std::vector<std::uint8_t> mask(width * height, 0);

    for (std::size_t y = 0; y < height; ++y) {
        const std::size_t row_offset = y * step;

        for (std::size_t x = 0; x < width; ++x) {
            const std::uint8_t pixel =
                body_index_map.data[row_offset + x];

            if (pixel == target_index) {
                mask[y * width + x] = 1;
            }
        }
    }

    return mask;
}

}  // namespace vision::segmentation