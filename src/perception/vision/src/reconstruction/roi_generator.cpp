#include "vision/reconstruction/roi_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vision::reconstruction
{

ROI3D ROIGenerator::generate(
    const std::vector<geometry_msgs::msg::Point>& points,
    double margin,
    std::uint32_t object_id,
    const std::string& object_type) const
{
    ROI3D roi;

    roi.object_id = object_id;
    roi.object_type = object_type;

    if (points.empty() || !std::isfinite(margin) || margin < 0.0) {
        return roi;
    }

    const double inf = std::numeric_limits<double>::infinity();

    roi.min.x = inf;
    roi.min.y = inf;
    roi.min.z = inf;

    roi.max.x = -inf;
    roi.max.y = -inf;
    roi.max.z = -inf;

    bool has_valid_point = false;

    for (const auto& point : points) {
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }

        roi.min.x = std::min(roi.min.x, point.x);
        roi.min.y = std::min(roi.min.y, point.y);
        roi.min.z = std::min(roi.min.z, point.z);

        roi.max.x = std::max(roi.max.x, point.x);
        roi.max.y = std::max(roi.max.y, point.y);
        roi.max.z = std::max(roi.max.z, point.z);

        has_valid_point = true;
    }

    if (!has_valid_point) {
        return ROI3D{};
    }

    roi.min.x -= margin;
    roi.min.y -= margin;
    roi.min.z -= margin;

    roi.max.x += margin;
    roi.max.y += margin;
    roi.max.z += margin;

    roi.valid =
        std::isfinite(roi.min.x) &&
        std::isfinite(roi.min.y) &&
        std::isfinite(roi.min.z) &&
        std::isfinite(roi.max.x) &&
        std::isfinite(roi.max.y) &&
        std::isfinite(roi.max.z);

    return roi;
}

}  // namespace vision::reconstruction