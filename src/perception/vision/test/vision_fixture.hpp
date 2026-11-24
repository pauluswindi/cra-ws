#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <map>
#include <set>
#include <string>

#include "vision/reconstruction/reconstruction_pipeline.hpp"

namespace vision_test
{

struct Fixture
{
    sensor_msgs::msg::Image depth;
    sensor_msgs::msg::Image body;
    sensor_msgs::msg::CameraInfo info;
    geometry_msgs::msg::TransformStamped transform;
    vision::reconstruction::ROI3D roi;
    std::map<int, vision::reconstruction::SkeletonData> skeletons;

    explicit Fixture(std::uint32_t width = 64, std::uint32_t height = 48,
                     bool floating = false, std::uint32_t padding = 0, int percent = 10)
    {
        depth.header.frame_id = body.header.frame_id = info.header.frame_id = "depth_camera";
        depth.header.stamp.sec = body.header.stamp.sec = info.header.stamp.sec = 10;
        depth.width = body.width = info.width = width;
        depth.height = body.height = info.height = height;
        depth.encoding = floating ? "32FC1" : "16UC1";
        body.encoding = "mono8";
        const std::size_t pixel_size = floating ? 4 : 2;
        depth.step = width * pixel_size + padding;
        body.step = width + padding;
        depth.data.resize(depth.step * height, 0);
        body.data.resize(body.step * height, 255);
        for (std::size_t y = 0; y < height; ++y) {
            for (std::size_t x = 0; x < width; ++x) {
                const auto index = y * width + x;
                if (static_cast<int>(index % 100) < percent) {
                    body.data[y * body.step + x] = 1;
                }
                if (floating) {
                    const float value = 1.f + static_cast<float>(index % 11) * .001f;
                    std::memcpy(&depth.data[y * depth.step + x * pixel_size], &value, sizeof(value));
                } else {
                    const std::uint16_t value = 1000 + index % 11;
                    std::memcpy(&depth.data[y * depth.step + x * pixel_size], &value, sizeof(value));
                }
            }
        }
        info.k = {static_cast<double>(width), 0., width / 2.,
                  0., static_cast<double>(height), height / 2., 0., 0., 1.};
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "depth_camera";
        transform.transform.rotation.w = 1.;
        roi.valid = true;
        roi.min.x = roi.min.y = -2.;
        roi.min.z = .3;
        roi.max.x = roi.max.y = 2.;
        roi.max.z = 3.;
        const std::map<int, std::array<double, 2>> joints{
            {0, {0., .15}}, {2, {0., -.1}}, {26, {0., -.25}},
            {5, {-.12, -.1}}, {6, {-.2, 0.}}, {7, {-.25, .1}}, {9, {-.28, .15}},
            {12, {.12, -.1}}, {13, {.2, 0.}}, {14, {.25, .1}}, {16, {.28, .15}}};
        for (const auto& [id, xy] : joints) {
            geometry_msgs::msg::Point point;
            point.x = xy[0]; point.y = xy[1]; point.z = 1.;
            skeletons[1].points.push_back(point);
            vision::human::BodyJoint joint;
            joint.id = id;
            joint.position = point;
            skeletons[1].joints.push_back(joint);
        }
    }
};

}  // namespace vision_test
