#include "vision/frame/frame_transformer.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace vision {
namespace frame {

FrameTransformer::FrameTransformer(rclcpp::Node* node)
: node_(node)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

FrameTransformer::~FrameTransformer() {}

bool FrameTransformer::lookupTransform(const std::string& target_frame,
                                       const std::string& source_frame,
                                       tf2::Transform& out) const {
    if (target_frame == source_frame) {
        out.setIdentity();
        return true;
    }
    try {
        const auto stamped = tf_buffer_->lookupTransform(
            target_frame, source_frame, tf2::TimePointZero);
        const auto& t = stamped.transform;
        out.setOrigin(tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
        out.setRotation(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w));
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "TF lookup failed %s to %s: %s",
                             source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

geometry_msgs::msg::Point FrameTransformer::transformPoint(
    const geometry_msgs::msg::Point& point_in,
    const std::string& target_frame,
    const std::string& source_frame)
{
    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.header.frame_id = source_frame;
    point_stamped.header.stamp = rclcpp::Time(0);
    point_stamped.point = point_in;

    try {
        auto transformed = tf_buffer_->transform(point_stamped, target_frame);
        return transformed.point;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Transform failed %s to %s: %s",
                             source_frame.c_str(), target_frame.c_str(), ex.what());
        return point_in;
    }
}

body_tracking::SkeletonData FrameTransformer::transformSkeleton(
    const body_tracking::SkeletonData& camera_skeleton,
    const std::string& target_frame,
    const std::string& source_frame)
{
    body_tracking::SkeletonData out = camera_skeleton;

    tf2::Transform tf;
    if (!lookupTransform(target_frame, source_frame, tf)) {
        return out;
    }

    const tf2::Quaternion rotation = tf.getRotation();
    for (size_t i = 0; i < out.joints.size(); ++i) {
        if (!out.is_tracked[i]) {
            continue;
        }

        const auto& p = out.joints[i];
        const tf2::Vector3 p_out = tf * tf2::Vector3(p.x, p.y, p.z);
        out.joints[i].x = p_out.x();
        out.joints[i].y = p_out.y();
        out.joints[i].z = p_out.z();

        const auto& o = out.orientations[i];
        tf2::Quaternion q_out = rotation * tf2::Quaternion(o.x, o.y, o.z, o.w);
        q_out.normalize();
        out.orientations[i].x = q_out.x();
        out.orientations[i].y = q_out.y();
        out.orientations[i].z = q_out.z();
        out.orientations[i].w = q_out.w();
    }

    out.frame_id = target_frame;
    return out;
}

bool FrameTransformer::transformMarkers(const visualization_msgs::msg::MarkerArray& markers_in,
                                        visualization_msgs::msg::MarkerArray& markers_out,
                                        const std::string& target_frame) {
    markers_out = visualization_msgs::msg::MarkerArray();
    if (markers_in.markers.empty()) {
        return true;
    }

    const std::string source_frame = markers_in.markers.front().header.frame_id;
    tf2::Transform tf;
    if (!lookupTransform(target_frame, source_frame, tf)) {
        return false;
    }

    const tf2::Quaternion rotation = tf.getRotation();
    markers_out.markers.reserve(markers_in.markers.size());

    for (const auto& marker : markers_in.markers) {
        auto transformed = marker;  // keep every driver field untouched

        const auto& p = marker.pose.position;
        const tf2::Vector3 p_out = tf * tf2::Vector3(p.x, p.y, p.z);
        transformed.pose.position.x = p_out.x();
        transformed.pose.position.y = p_out.y();
        transformed.pose.position.z = p_out.z();

        const auto& o = marker.pose.orientation;
        tf2::Quaternion q_out = rotation * tf2::Quaternion(o.x, o.y, o.z, o.w);
        q_out.normalize();
        transformed.pose.orientation.x = q_out.x();
        transformed.pose.orientation.y = q_out.y();
        transformed.pose.orientation.z = q_out.z();
        transformed.pose.orientation.w = q_out.w();

        transformed.header.frame_id = target_frame;
        markers_out.markers.push_back(transformed);
    }
    return true;
}

bool FrameTransformer::canTransform(const std::string& target_frame,
                                    const std::string& source_frame) const {
    tf2::Transform unused;
    return lookupTransform(target_frame, source_frame, unused);
}

} // namespace frame
} // namespace vision