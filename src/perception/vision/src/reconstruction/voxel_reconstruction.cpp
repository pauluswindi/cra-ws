#include "vision/reconstruction/voxel_reconstruction.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace vision::reconstruction
{

namespace
{

using VoxelKey = std::array<std::int64_t, 3>;

struct VoxelHash
{
    std::size_t operator()(const VoxelKey& key) const
    {
        std::size_t seed = 0;

        for (const auto index : key) {
            const std::size_t value = std::hash<std::int64_t>{}(index);

            seed ^= value + static_cast<std::size_t>(0x9e3779b9)
                    + (seed << 6) + (seed >> 2);
        }

        return seed;
    }
};

std::int64_t voxelIndex(double coordinate, double voxel_size)
{
    const double index = std::floor(coordinate / voxel_size);

    const long double value = static_cast<long double>(index);

    if (!std::isfinite(index) ||
        value < static_cast<long double>(
            std::numeric_limits<std::int64_t>::min()) ||
        value >= -static_cast<long double>(
            std::numeric_limits<std::int64_t>::min())) {
        throw std::out_of_range("Voxel index out of range");
    }

    return static_cast<std::int64_t>(index);
}

}  // namespace

std::vector<Voxel> VoxelReconstruction::reconstruct(
    const std::vector<geometry_msgs::msg::Point>& points,
    double voxel_size) const
{
    if (!std::isfinite(voxel_size) || voxel_size <= 0.0) {
        throw std::invalid_argument("Invalid voxel size");
    }

    std::unordered_set<VoxelKey, VoxelHash> occupied;
    occupied.reserve(points.size());

    for (const auto& point : points) {
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }

        const VoxelKey key = {
            voxelIndex(point.x, voxel_size),
            voxelIndex(point.y, voxel_size),
            voxelIndex(point.z, voxel_size)
        };

        occupied.insert(key);
    }

    std::vector<Voxel> voxels;
    voxels.reserve(occupied.size());

    for (const auto& key : occupied) {
        Voxel voxel;

        voxel.x = key[0];
        voxel.y = key[1];
        voxel.z = key[2];

        voxel.center.x =
            (static_cast<double>(voxel.x) + 0.5) * voxel_size;

        voxel.center.y =
            (static_cast<double>(voxel.y) + 0.5) * voxel_size;

        voxel.center.z =
            (static_cast<double>(voxel.z) + 0.5) * voxel_size;

        if (!std::isfinite(voxel.center.x) ||
            !std::isfinite(voxel.center.y) ||
            !std::isfinite(voxel.center.z)) {
            throw std::out_of_range("Voxel center out of range");
        }

        voxels.push_back(voxel);
    }

    std::sort(
        voxels.begin(),
        voxels.end(),
        [](const Voxel& a, const Voxel& b) {
            if (a.x != b.x) {
                return a.x < b.x;
            }

            if (a.y != b.y) {
                return a.y < b.y;
            }

            return a.z < b.z;
        });

    return voxels;
}

}  // namespace vision::reconstruction