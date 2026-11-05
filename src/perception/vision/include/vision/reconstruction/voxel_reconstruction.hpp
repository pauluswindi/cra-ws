#pragma once

#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace vision::reconstruction
{

struct Voxel
{
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    geometry_msgs::msg::Point center;
};

class VoxelReconstruction
{
public:
    std::vector<Voxel> reconstruct(
        const std::vector<geometry_msgs::msg::Point>& points,
        double voxel_size) const;
};

}  // namespace vision::reconstruction