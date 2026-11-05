#include "vision/reconstruction/depth_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "sensor_msgs/image_encodings.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"

namespace vision::reconstruction
{

void DepthProcessor::updateLookupTable(
    const sensor_msgs::msg::CameraInfo& camera_info) const
{
    if (lookup_ready_ &&
        cached_camera_info_.width == camera_info.width &&
        cached_camera_info_.height == camera_info.height &&
        cached_camera_info_.k == camera_info.k &&
        cached_camera_info_.d == camera_info.d &&
        cached_camera_info_.distortion_model ==
            camera_info.distortion_model) {
        return;
    }

    if (camera_info.width == 0 || camera_info.height == 0) {
        throw std::invalid_argument("Invalid camera resolution");
    }

    for (const double value : camera_info.k) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("Invalid camera matrix");
        }
    }

    const double fx = camera_info.k[0];
    const double fy = camera_info.k[4];
    const double cx = camera_info.k[2];
    const double cy = camera_info.k[5];

    if (fx <= 0.0 || fy <= 0.0) {
        throw std::invalid_argument("Invalid camera intrinsics");
    }

    bool has_distortion = false;

    for (const double coefficient : camera_info.d) {
        if (!std::isfinite(coefficient)) {
            throw std::invalid_argument(
                "Invalid distortion coefficients");
        }

        if (std::abs(coefficient) > 1e-12) {
            has_distortion = true;
        }
    }

    const auto& model = camera_info.distortion_model;

    if (has_distortion) {
        if (model == "plumb_bob") {
            if (camera_info.d.size() != 5) {
                throw std::invalid_argument(
                    "plumb_bob requires 5 distortion coefficients");
            }
        } else if (model == "rational_polynomial") {
            if (camera_info.d.size() != 8) {
                throw std::invalid_argument(
                    "rational_polynomial requires 8 distortion coefficients");
            }
        } else {
            throw std::invalid_argument(
                "Unsupported distortion model: " + model);
        }
    }

    const cv::Matx33d camera_matrix(
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);

    const std::size_t width = camera_info.width;
    const std::size_t height = camera_info.height;

    std::vector<cv::Point2f> distorted_pixels;
    distorted_pixels.reserve(width * height);

    for (std::size_t v = 0; v < height; ++v) {
        for (std::size_t u = 0; u < width; ++u) {
            distorted_pixels.emplace_back(
                static_cast<float>(u),
                static_cast<float>(v));
        }
    }

    std::vector<cv::Point2f> corrected_rays;

    if (has_distortion) {
        const cv::Mat distortion_coefficients(camera_info.d);

        cv::undistortPoints(
            distorted_pixels,
            corrected_rays,
            cv::Mat(camera_matrix),
            distortion_coefficients);
    } else {
        corrected_rays.reserve(distorted_pixels.size());

        for (const auto& pixel : distorted_pixels) {
            corrected_rays.emplace_back(
                static_cast<float>((pixel.x - cx) / fx),
                static_cast<float>((pixel.y - cy) / fy));
        }
    }

    if (corrected_rays.size() != distorted_pixels.size()) {
        throw std::runtime_error(
            "Failed to generate depth lookup table");
    }

    undistorted_rays_ = std::move(corrected_rays);
    cached_camera_info_ = camera_info;
    lookup_ready_ = true;
}

std::vector<geometry_msgs::msg::Point> DepthProcessor::extractPoints(
    const sensor_msgs::msg::Image& depth_image,
    const sensor_msgs::msg::CameraInfo& camera_info,
    const geometry_msgs::msg::TransformStamped& transform,
    const ROI3D& roi,
    double min_depth,
    double max_depth,
    const std::vector<std::uint8_t>* pixel_mask) const
{
    std::vector<geometry_msgs::msg::Point> points;

    if (!roi.valid) {
        return points;
    }

    if (!std::isfinite(min_depth) ||
        !std::isfinite(max_depth) ||
        min_depth <= 0.0 ||
        max_depth <= min_depth) {
        throw std::invalid_argument("Invalid depth range");
    }

    if (depth_image.header.frame_id.empty() ||
        depth_image.header.frame_id != camera_info.header.frame_id ||
        depth_image.header.frame_id != transform.child_frame_id ||
        transform.header.frame_id != "base_link") {
        throw std::invalid_argument(
            "Inconsistent coordinate frames");
    }

    if (depth_image.width != camera_info.width ||
        depth_image.height != camera_info.height ||
        depth_image.width == 0 ||
        depth_image.height == 0) {
        throw std::invalid_argument(
            "Depth image and CameraInfo mismatch");
    }

    const std::size_t width = depth_image.width;
    const std::size_t height = depth_image.height;

    if (pixel_mask != nullptr &&
        pixel_mask->size() != width * height) {
        throw std::invalid_argument(
            "Pixel mask dimensions do not match depth image");
    }

    const bool depth_u16 =
        depth_image.encoding ==
        sensor_msgs::image_encodings::TYPE_16UC1;

    const bool depth_f32 =
        depth_image.encoding ==
        sensor_msgs::image_encodings::TYPE_32FC1;

    if (!depth_u16 && !depth_f32) {
        throw std::invalid_argument(
            "Unsupported depth encoding: " +
            depth_image.encoding);
    }

    const std::size_t pixel_size = depth_u16 ? 2 : 4;
    const std::size_t row_size = width * pixel_size;

    if (depth_image.step < row_size ||
        depth_image.data.size() /
            static_cast<std::size_t>(depth_image.step) < height) {
        throw std::invalid_argument(
            "Invalid depth image buffer");
    }

    const std::uint16_t endian_test = 1;

    const bool host_big_endian =
        *reinterpret_cast<const std::uint8_t*>(
            &endian_test) == 0;

    if ((depth_image.is_bigendian != 0) != host_big_endian) {
        throw std::invalid_argument(
            "Unsupported depth byte order");
    }

    const auto& translation = transform.transform.translation;
    const auto& rotation = transform.transform.rotation;

    if (!std::isfinite(translation.x) ||
        !std::isfinite(translation.y) ||
        !std::isfinite(translation.z) ||
        !std::isfinite(rotation.x) ||
        !std::isfinite(rotation.y) ||
        !std::isfinite(rotation.z) ||
        !std::isfinite(rotation.w)) {
        throw std::invalid_argument("Invalid transform");
    }

    tf2::Quaternion quaternion(
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w);

    if (!std::isfinite(quaternion.length2()) ||
        quaternion.length2() < 1e-12) {
        throw std::invalid_argument(
            "Invalid transform quaternion");
    }

    quaternion.normalize();

    const tf2::Transform camera_to_base(
        quaternion,
        tf2::Vector3(
            translation.x,
            translation.y,
            translation.z));

    const tf2::Transform base_to_camera =
        camera_to_base.inverse();

    const double infinity =
        std::numeric_limits<double>::infinity();

    tf2::Vector3 camera_min(
        infinity, infinity, infinity);

    tf2::Vector3 camera_max(
        -infinity, -infinity, -infinity);

    if (!std::isfinite(roi.min.x) ||
        !std::isfinite(roi.min.y) ||
        !std::isfinite(roi.min.z) ||
        !std::isfinite(roi.max.x) ||
        !std::isfinite(roi.max.y) ||
        !std::isfinite(roi.max.z) ||
        roi.min.x > roi.max.x ||
        roi.min.y > roi.max.y ||
        roi.min.z > roi.max.z) {
        throw std::invalid_argument("Invalid ROI bounds");
    }

    for (int i = 0; i < 8; ++i) {
        const tf2::Vector3 corner(
            (i & 1) ? roi.max.x : roi.min.x,
            (i & 2) ? roi.max.y : roi.min.y,
            (i & 4) ? roi.max.z : roi.min.z);

        const tf2::Vector3 camera_corner =
            base_to_camera * corner;

        for (int axis = 0; axis < 3; ++axis) {
            camera_min[axis] = std::min(
                camera_min[axis],
                camera_corner[axis]);

            camera_max[axis] = std::max(
                camera_max[axis],
                camera_corner[axis]);
        }
    }

    const double near_depth =
        std::max(min_depth, camera_min.z());

    const double far_depth =
        std::min(max_depth, camera_max.z());

    if (near_depth > far_depth) {
        return points;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    updateLookupTable(camera_info);

    const std::uint8_t* data = depth_image.data.data();

    for (std::size_t v = 0; v < height; ++v) {
        const std::size_t row_offset =
            v * depth_image.step;

        for (std::size_t u = 0; u < width; ++u) {
            const std::size_t index = v * width + u;

            // Ignore pixels outside the selected object mask.
            if (pixel_mask != nullptr &&
                (*pixel_mask)[index] == 0) {
                continue;
            }

            const std::size_t offset =
                row_offset + u * pixel_size;

            double depth = 0.0;

            if (depth_u16) {
                std::uint16_t value;

                std::memcpy(
                    &value,
                    data + offset,
                    sizeof(value));

                depth =
                    static_cast<double>(value) * 0.001;
            } else {
                float value;

                std::memcpy(
                    &value,
                    data + offset,
                    sizeof(value));

                depth = static_cast<double>(value);
            }

            if (!std::isfinite(depth) ||
                depth < near_depth ||
                depth > far_depth) {
                continue;
            }

            const auto& ray = undistorted_rays_[index];

            if (!std::isfinite(ray.x) ||
                !std::isfinite(ray.y)) {
                continue;
            }

            const double x =
                static_cast<double>(ray.x) * depth;

            const double y =
                static_cast<double>(ray.y) * depth;

            if (x < camera_min.x() ||
                x > camera_max.x() ||
                y < camera_min.y() ||
                y > camera_max.y()) {
                continue;
            }

            const tf2::Vector3 point_base =
                camera_to_base * tf2::Vector3(x, y, depth);

            if (point_base.x() < roi.min.x ||
                point_base.x() > roi.max.x ||
                point_base.y() < roi.min.y ||
                point_base.y() > roi.max.y ||
                point_base.z() < roi.min.z ||
                point_base.z() > roi.max.z) {
                continue;
            }

            geometry_msgs::msg::Point point;
            point.x = point_base.x();
            point.y = point_base.y();
            point.z = point_base.z();

            points.push_back(point);
        }
    }

    return points;
}

}  // namespace vision::reconstruction