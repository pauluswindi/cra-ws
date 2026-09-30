#ifndef VISION__FRAME__FRAME_TRANSFORMER_HPP_
#define VISION__FRAME__FRAME_TRANSFORMER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace vision {
namespace frame {

class FrameTransformer {
public:
    explicit FrameTransformer(rclcpp::Node* node);
    ~FrameTransformer() = default;

    geometry_msgs::msg::Point transformPoint(
        const geometry_msgs::msg::Point& point_in,
        const std::string& target_frame,
        const std::string& source_frame) const;

    bool transformMarkers(
        const visualization_msgs::msg::MarkerArray& markers_in,
        visualization_msgs::msg::MarkerArray& markers_out,
        const std::string& target_frame) const;

    bool canTransform(const std::string& target_frame, const std::string& source_frame) const;

private:
    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         tf2::Transform& out) const;

    rclcpp::Node* node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace frame
}  // namespace vision

#endif  // VISION__FRAME__FRAME_TRANSFORMER_HPP_