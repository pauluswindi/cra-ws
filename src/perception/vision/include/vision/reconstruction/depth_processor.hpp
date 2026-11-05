#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include <opencv2/core/types.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "vision/reconstruction/roi.hpp"

namespace vision::reconstruction
{

class DepthProcessor
{
public:
    std::vector<geometry_msgs::msg::Point> extractPoints(
        const sensor_msgs::msg::Image& depth_image,
        const sensor_msgs::msg::CameraInfo& camera_info,
        const geometry_msgs::msg::TransformStamped& transform,
        const ROI3D& roi,
        double min_depth,
        double max_depth,
        const std::vector<std::uint8_t>* pixel_mask = nullptr) const;

private:
    void updateLookupTable(
        const sensor_msgs::msg::CameraInfo& camera_info) const;

    mutable std::mutex mutex_;
    mutable bool lookup_ready_ = false;

    mutable sensor_msgs::msg::CameraInfo cached_camera_info_;
    mutable std::vector<cv::Point2f> undistorted_rays_;
};

}  // namespace vision::reconstruction