#include "vision/segmentation/human_segmenter.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "sensor_msgs/image_encodings.hpp"

namespace vision::segmentation
{

namespace
{

void validateBodyIndex(const sensor_msgs::msg::Image& body_index_map)
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
}

}  // namespace

std::vector<std::uint8_t> HumanSegmenter::generateMask(
    const sensor_msgs::msg::Image& body_index_map,
    std::uint32_t body_id) const
{
    validateBodyIndex(body_index_map);
    const std::size_t width = body_index_map.width;
    const std::size_t height = body_index_map.height;
    const std::size_t step = body_index_map.step;

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

std::vector<PixelRun> HumanSegmenter::generatePixelRuns(
    const sensor_msgs::msg::Image& body_index_map,
    std::uint32_t body_id) const
{
    validateBodyIndex(body_index_map);
    const std::size_t width = body_index_map.width;
    const std::size_t height = body_index_map.height;
    const auto target = static_cast<std::uint8_t>(body_id % 255);
    std::vector<PixelRun> runs;
    std::size_t begin = 0;
    bool open = false;
    for (std::size_t y = 0; y < height; ++y) {
        const std::size_t offset = y * body_index_map.step;
        for (std::size_t x = 0; x < width; ++x) {
            const auto index = y * width + x;
            if (body_index_map.data[offset + x] == target) {
                if (!open) {
                    begin = index;
                    open = true;
                }
            } else if (open) {
                runs.push_back({begin, index});
                open = false;
            }
        }
    }
    if (open) {
        runs.push_back({begin, width * height});
    }
    return runs;
}

}  // namespace vision::segmentation
