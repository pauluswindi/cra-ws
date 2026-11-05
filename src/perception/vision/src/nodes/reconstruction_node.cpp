#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "vision/human/human_capsule.hpp"
#include "vision/reconstruction/depth_processor.hpp"
#include "vision/reconstruction/roi_generator.hpp"
#include "vision/reconstruction/voxel_reconstruction.hpp"
#include "vision/segmentation/human_segmenter.hpp"

namespace
{

constexpr char kBaseFrame[] = "base_link";
constexpr char kZoneNamespace[] = "humans_in_zone";
constexpr char kVoxelNamespace[] = "human_voxels";
constexpr char kCapsuleNamespace[] = "human_capsule_";

constexpr int kQueueSize = 10;

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Point = geometry_msgs::msg::Point;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

bool isUpperBodyJoint(int joint_id)
{
    return (joint_id >= 0 && joint_id <= 17) ||
           (joint_id >= 26 && joint_id <= 31);
}

bool isValidPoint(const Point& point)
{
    return std::isfinite(point.x) &&
           std::isfinite(point.y) &&
           std::isfinite(point.z);
}

double stampDifference(
    const builtin_interfaces::msg::Time& first,
    const builtin_interfaces::msg::Time& second)
{
    return std::abs(
        (rclcpp::Time(first) - rclcpp::Time(second)).seconds());
}

}  // namespace

class ReconstructionNode : public rclcpp::Node
{
public:
    ReconstructionNode()
        : Node("reconstruction_node"),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_)
    {
        const auto depth_topic = declare_parameter<std::string>(
            "depth_topic", "/depth/image_raw");

        const auto camera_info_topic = declare_parameter<std::string>(
            "camera_info_topic", "/depth/camera_info");

        const auto body_index_topic = declare_parameter<std::string>(
            "body_index_topic", "/body_index_map/image_raw");

        roi_margin_ = declare_parameter<double>(
            "roi_margin", 0.15);

        voxel_size_ = declare_parameter<double>(
            "voxel_size", 0.02);

        min_depth_ = declare_parameter<double>(
            "min_depth", 0.3);

        max_depth_ = declare_parameter<double>(
            "max_depth", 3.0);

        max_sync_delay_ = declare_parameter<double>(
            "max_sync_delay", 0.15);

        data_timeout_ = declare_parameter<double>(
            "data_timeout", 0.5);

        capsule_config_.torso_radius = declare_parameter<double>(
            "capsule.torso_radius", 0.13);

        capsule_config_.head_radius = declare_parameter<double>(
            "capsule.head_radius", 0.09);

        capsule_config_.upper_arm_radius = declare_parameter<double>(
            "capsule.upper_arm_radius", 0.055);

        capsule_config_.forearm_radius = declare_parameter<double>(
            "capsule.forearm_radius", 0.045);

        capsule_config_.torso_max_radius = declare_parameter<double>(
            "capsule.torso_max_radius", 0.22);

        capsule_config_.head_max_radius = declare_parameter<double>(
            "capsule.head_max_radius", 0.14);

        capsule_config_.upper_arm_max_radius = declare_parameter<double>(
            "capsule.upper_arm_max_radius", 0.10);

        capsule_config_.forearm_max_radius = declare_parameter<double>(
            "capsule.forearm_max_radius", 0.085);

        capsule_config_.safety_margin = declare_parameter<double>(
            "capsule.safety_margin", 0.015);

        capsule_config_.radius_percentile = declare_parameter<double>(
            "capsule.radius_percentile", 0.85);

        const int min_voxels = declare_parameter<int>(
            "capsule.min_voxels", 10);

        if (min_voxels < 1) {
            throw std::invalid_argument(
                "capsule.min_voxels must be positive");
        }

        capsule_config_.min_voxels =
            static_cast<std::size_t>(min_voxels);

        if (!std::isfinite(roi_margin_) ||
            !std::isfinite(voxel_size_) ||
            !std::isfinite(min_depth_) ||
            !std::isfinite(max_depth_) ||
            !std::isfinite(max_sync_delay_) ||
            !std::isfinite(data_timeout_) ||
            roi_margin_ < 0.0 ||
            voxel_size_ <= 0.0 ||
            min_depth_ <= 0.0 ||
            max_depth_ <= min_depth_ ||
            max_sync_delay_ < 0.0 ||
            data_timeout_ <= 0.0) {
            throw std::invalid_argument(
                "Invalid reconstruction configuration");
        }

        const auto sensor_qos = rclcpp::SensorDataQoS();

        depth_sub_ = create_subscription<Image>(
            depth_topic,
            sensor_qos,
            [this](Image::ConstSharedPtr msg) {
                depthCallback(msg);
            });

        camera_info_sub_ = create_subscription<CameraInfo>(
            camera_info_topic,
            sensor_qos,
            [this](CameraInfo::ConstSharedPtr msg) {
                cameraInfoCallback(msg);
            });

        body_index_sub_ = create_subscription<Image>(
            body_index_topic,
            sensor_qos,
            [this](Image::ConstSharedPtr msg) {
                bodyIndexCallback(msg);
            });

        skeleton_sub_ = create_subscription<MarkerArray>(
            "/vision/skeleton/base_frame",
            kQueueSize,
            [this](MarkerArray::ConstSharedPtr msg) {
                skeletonCallback(msg);
            });

        zone_sub_ = create_subscription<MarkerArray>(
            "/perception/humans_in_zone",
            kQueueSize,
            [this](MarkerArray::ConstSharedPtr msg) {
                zoneCallback(msg);
            });

        voxel_pub_ = create_publisher<MarkerArray>(
            "/vision/reconstruction/voxels",
            kQueueSize);

        capsule_pub_ = create_publisher<MarkerArray>(
            "/vision/human/capsules",
            kQueueSize);

        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() {
                watchdogCallback();
            });

        RCLCPP_INFO(
            get_logger(),
            "Hybrid reconstruction started: voxel=%.3f m, ROI=%.3f m",
            voxel_size_,
            roi_margin_);
    }

private:
    void depthCallback(Image::ConstSharedPtr msg)
    {
        depth_image_ = std::move(msg);
        last_depth_received_ = now();

        processFrame();
    }

    void cameraInfoCallback(CameraInfo::ConstSharedPtr msg)
    {
        camera_info_ = std::move(msg);
    }

    void bodyIndexCallback(Image::ConstSharedPtr msg)
    {
        body_index_map_ = std::move(msg);
        last_body_index_received_ = now();

        processFrame();
    }

    void skeletonCallback(MarkerArray::ConstSharedPtr msg)
    {
        skeleton_points_.clear();
        skeleton_joints_.clear();
        skeleton_stamps_.clear();

        for (const auto& marker : msg->markers) {
            if (marker.action != Marker::ADD ||
                marker.id < 0 ||
                marker.header.frame_id != kBaseFrame) {
                continue;
            }

            const int body_id = marker.id / 100;
            const int joint_id = marker.id % 100;

            if (!isUpperBodyJoint(joint_id) ||
                !isValidPoint(marker.pose.position)) {
                continue;
            }

            skeleton_points_[body_id].push_back(
                marker.pose.position);

            vision::human::BodyJoint joint;
            joint.id = joint_id;
            joint.position = marker.pose.position;

            skeleton_joints_[body_id].push_back(joint);

            skeleton_stamps_[body_id] = marker.header.stamp;
        }

        last_skeleton_received_ = now();

        processFrame();
    }

    void zoneCallback(MarkerArray::ConstSharedPtr msg)
    {
        active_body_ids_.clear();
        zone_stamps_.clear();

        for (const auto& marker : msg->markers) {
            if (marker.ns != kZoneNamespace ||
                marker.action != Marker::ADD ||
                marker.id < 0 ||
                marker.header.frame_id != kBaseFrame) {
                continue;
            }

            const int body_id = marker.id / 100;

            active_body_ids_.insert(body_id);
            zone_stamps_[body_id] = marker.header.stamp;
        }

        last_zone_received_ = now();

        if (active_body_ids_.empty()) {
            clearOutputs();
            return;
        }

        processFrame();
    }

    bool isRecent(const rclcpp::Time& received) const
    {
        if (received.nanoseconds() == 0) {
            return false;
        }

        const double age = (now() - received).seconds();

        return age >= 0.0 && age <= data_timeout_;
    }

    bool inputsReady() const
    {
        if (!depth_image_ ||
            !camera_info_ ||
            !body_index_map_ ||
            active_body_ids_.empty()) {
            return false;
        }

        return isRecent(last_depth_received_) &&
               isRecent(last_body_index_received_) &&
               isRecent(last_skeleton_received_) &&
               isRecent(last_zone_received_);
    }

    void appendCapsuleMarkers(
        const std::vector<vision::human::Capsule>& capsules,
        const builtin_interfaces::msg::Time& stamp,
        MarkerArray& output,
        std::set<std::pair<std::string, int>>& current_ids) const
    {
        for (const auto& capsule : capsules) {
            const auto& a = capsule.start;
            const auto& b = capsule.end;

            if (!isValidPoint(a) ||
                !isValidPoint(b) ||
                !std::isfinite(capsule.radius) ||
                capsule.radius <= 0.0) {
                continue;
            }

            const tf2::Vector3 start(a.x, a.y, a.z);
            const tf2::Vector3 end(b.x, b.y, b.z);
            const tf2::Vector3 direction = end - start;

            const double length = direction.length();

            if (!std::isfinite(length)) {
                continue;
            }

            const double diameter = capsule.radius * 2.0;

            Marker marker;
            marker.header.frame_id = kBaseFrame;
            marker.header.stamp = stamp;

            marker.ns =
                std::string(kCapsuleNamespace) +
                std::to_string(capsule.body_id);

            marker.action = Marker::ADD;

            marker.color.r = 1.0f;
            marker.color.g = 0.55f;
            marker.color.b = 0.1f;
            marker.color.a = 0.25f;

            marker.lifetime.sec = 0;
            marker.lifetime.nanosec = 500000000;

            marker.scale.x = diameter;
            marker.scale.y = diameter;

            if (length < 1e-6) {
                marker.id = capsule.segment_id * 3;
                marker.type = Marker::SPHERE;

                marker.pose.position = a;
                marker.pose.orientation.w = 1.0;
                marker.scale.z = diameter;

                current_ids.emplace(marker.ns, marker.id);
                output.markers.push_back(marker);
                continue;
            }

            const tf2::Vector3 normalized = direction / length;
            const tf2::Vector3 z_axis(0.0, 0.0, 1.0);

            const double cosine = std::clamp(
                normalized.z(), -1.0, 1.0);

            const tf2::Vector3 rotation_axis =
                z_axis.cross(normalized);

            tf2::Quaternion rotation;

            if (rotation_axis.length2() > 1e-12) {
                rotation.setRotation(
                    rotation_axis.normalized(),
                    std::acos(cosine));
            } else if (cosine < 0.0) {
                rotation.setRotation(
                    tf2::Vector3(1.0, 0.0, 0.0),
                    std::acos(-1.0));
            } else {
                rotation.setValue(0.0, 0.0, 0.0, 1.0);
            }

            rotation.normalize();

            marker.id = capsule.segment_id * 3;
            marker.type = Marker::CYLINDER;

            marker.pose.position.x = (a.x + b.x) * 0.5;
            marker.pose.position.y = (a.y + b.y) * 0.5;
            marker.pose.position.z = (a.z + b.z) * 0.5;

            marker.pose.orientation.x = rotation.x();
            marker.pose.orientation.y = rotation.y();
            marker.pose.orientation.z = rotation.z();
            marker.pose.orientation.w = rotation.w();

            marker.scale.z = length;

            current_ids.emplace(marker.ns, marker.id);
            output.markers.push_back(marker);

            marker.type = Marker::SPHERE;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.z = diameter;

            marker.id = capsule.segment_id * 3 + 1;
            marker.pose.position = a;

            current_ids.emplace(marker.ns, marker.id);
            output.markers.push_back(marker);

            marker.id = capsule.segment_id * 3 + 2;
            marker.pose.position = b;

            current_ids.emplace(marker.ns, marker.id);
            output.markers.push_back(marker);
        }
    }

    void processFrame()
    {
        if (!inputsReady()) {
            return;
        }

        const auto& depth = *depth_image_;
        const auto& body_map = *body_index_map_;
        const auto& camera_info = *camera_info_;

        if (stampDifference(
                depth.header.stamp,
                body_map.header.stamp) > max_sync_delay_) {
            clearOutputs();
            return;
        }

        if (depth.width != body_map.width ||
            depth.height != body_map.height ||
            depth.header.frame_id != body_map.header.frame_id ||
            depth.header.frame_id != camera_info.header.frame_id) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Depth and Body Index Map geometry/frame mismatch");

            clearOutputs();
            return;
        }

        const auto frame_stamp = rclcpp::Time(depth.header.stamp);

        if (frame_stamp == last_processed_stamp_) {
            return;
        }

        std::set<std::uint8_t> used_indices;

        for (const int body_id : active_body_ids_) {
            const auto index =
                static_cast<std::uint8_t>(body_id % 255);

            if (!used_indices.insert(index).second) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Ambiguous human body index detected");

                clearOutputs();
                return;
            }
        }

        for (const int body_id : active_body_ids_) {
            const auto skeleton_stamp =
                skeleton_stamps_.find(body_id);

            const auto zone_stamp =
                zone_stamps_.find(body_id);

            if (skeleton_stamp == skeleton_stamps_.end() ||
                zone_stamp == zone_stamps_.end() ||
                stampDifference(
                    depth.header.stamp,
                    skeleton_stamp->second) > max_sync_delay_ ||
                stampDifference(
                    depth.header.stamp,
                    zone_stamp->second) > max_sync_delay_) {
                clearOutputs();
                return;
            }
        }

        geometry_msgs::msg::TransformStamped transform;

        try {
            transform = tf_buffer_.lookupTransform(
                kBaseFrame,
                depth.header.frame_id,
                depth.header.stamp,
                rclcpp::Duration::from_seconds(0.03));
        } catch (const tf2::TransformException& error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Depth TF unavailable: %s",
                error.what());

            clearOutputs();
            return;
        }

        MarkerArray voxel_output;
        MarkerArray capsule_output;

        std::set<int> current_voxel_ids;
        std::set<std::pair<std::string, int>> current_capsule_ids;

        try {
            for (const int body_id : active_body_ids_) {
                const auto skeleton_it =
                    skeleton_points_.find(body_id);

                const auto joints_it =
                    skeleton_joints_.find(body_id);

                if (skeleton_it == skeleton_points_.end() ||
                    skeleton_it->second.empty() ||
                    joints_it == skeleton_joints_.end()) {
                    continue;
                }

                const auto roi = roi_generator_.generate(
                    skeleton_it->second,
                    roi_margin_,
                    static_cast<std::uint32_t>(body_id),
                    "human");

                if (!roi.valid) {
                    continue;
                }

                const auto human_mask =
                    human_segmenter_.generateMask(
                        body_map,
                        static_cast<std::uint32_t>(body_id));

                const auto points =
                    depth_processor_.extractPoints(
                        depth,
                        camera_info,
                        transform,
                        roi,
                        min_depth_,
                        max_depth_,
                        &human_mask);

                const auto voxels =
                    voxel_reconstruction_.reconstruct(
                        points,
                        voxel_size_);

                const auto capsules = human_capsule_.generate(
                    joints_it->second,
                    voxels,
                    static_cast<std::uint32_t>(body_id),
                    capsule_config_);

                appendCapsuleMarkers(
                    capsules,
                    depth.header.stamp,
                    capsule_output,
                    current_capsule_ids);

                if (voxels.empty()) {
                    continue;
                }

                Marker marker;
                marker.header.frame_id = kBaseFrame;
                marker.header.stamp = depth.header.stamp;

                marker.ns = kVoxelNamespace;
                marker.id = body_id;
                marker.type = Marker::CUBE_LIST;
                marker.action = Marker::ADD;

                marker.pose.orientation.w = 1.0;

                marker.scale.x = voxel_size_;
                marker.scale.y = voxel_size_;
                marker.scale.z = voxel_size_;

                marker.color.r = 0.1f;
                marker.color.g = 0.8f;
                marker.color.b = 1.0f;
                marker.color.a = 0.9f;

                marker.points.reserve(voxels.size());

                for (const auto& voxel : voxels) {
                    marker.points.push_back(voxel.center);
                }

                voxel_output.markers.push_back(std::move(marker));
                current_voxel_ids.insert(body_id);
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Hybrid reconstruction failed: %s",
                error.what());

            clearOutputs();
            return;
        }

        // Delete voxel markers that are no longer valid.
        for (const int old_id : published_voxel_ids_) {
            if (current_voxel_ids.count(old_id) != 0) {
                continue;
            }

            Marker marker;
            marker.header.frame_id = kBaseFrame;
            marker.header.stamp = depth.header.stamp;
            marker.ns = kVoxelNamespace;
            marker.id = old_id;
            marker.action = Marker::DELETE;

            voxel_output.markers.push_back(marker);
        }

        // Delete capsule markers that are no longer valid.
        for (const auto& old_id : published_capsule_ids_) {
            if (current_capsule_ids.count(old_id) != 0) {
                continue;
            }

            Marker marker;
            marker.header.frame_id = kBaseFrame;
            marker.header.stamp = depth.header.stamp;
            marker.ns = old_id.first;
            marker.id = old_id.second;
            marker.action = Marker::DELETE;

            capsule_output.markers.push_back(marker);
        }

        published_voxel_ids_ = std::move(current_voxel_ids);
        published_capsule_ids_ = std::move(current_capsule_ids);

        last_processed_stamp_ = frame_stamp;

        if (!voxel_output.markers.empty()) {
            voxel_pub_->publish(voxel_output);
        }

        if (!capsule_output.markers.empty()) {
            capsule_pub_->publish(capsule_output);
        }
    }

    void clearVoxels()
    {
        if (published_voxel_ids_.empty()) {
            return;
        }

        MarkerArray output;

        for (const int body_id : published_voxel_ids_) {
            Marker marker;
            marker.header.frame_id = kBaseFrame;
            marker.header.stamp = now();
            marker.ns = kVoxelNamespace;
            marker.id = body_id;
            marker.action = Marker::DELETE;

            output.markers.push_back(marker);
        }

        voxel_pub_->publish(output);
        published_voxel_ids_.clear();
    }

    void clearCapsules()
    {
        if (published_capsule_ids_.empty()) {
            return;
        }

        MarkerArray output;

        for (const auto& id : published_capsule_ids_) {
            Marker marker;
            marker.header.frame_id = kBaseFrame;
            marker.header.stamp = now();
            marker.ns = id.first;
            marker.id = id.second;
            marker.action = Marker::DELETE;

            output.markers.push_back(marker);
        }

        capsule_pub_->publish(output);
        published_capsule_ids_.clear();
    }

    void clearOutputs()
    {
        clearVoxels();
        clearCapsules();
    }

    void watchdogCallback()
    {
        if (!isRecent(last_depth_received_) ||
            !isRecent(last_body_index_received_) ||
            !isRecent(last_skeleton_received_) ||
            !isRecent(last_zone_received_)) {
            clearOutputs();
        }
    }

    double roi_margin_;
    double voxel_size_;
    double min_depth_;
    double max_depth_;
    double max_sync_delay_;
    double data_timeout_;

    vision::reconstruction::ROIGenerator roi_generator_;
    vision::reconstruction::DepthProcessor depth_processor_;
    vision::reconstruction::VoxelReconstruction voxel_reconstruction_;

    vision::segmentation::HumanSegmenter human_segmenter_;

    vision::human::HumanCapsule human_capsule_;
    vision::human::CapsuleConfig capsule_config_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<Image>::SharedPtr body_index_sub_;
    rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Subscription<MarkerArray>::SharedPtr skeleton_sub_;
    rclcpp::Subscription<MarkerArray>::SharedPtr zone_sub_;

    rclcpp::Publisher<MarkerArray>::SharedPtr voxel_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr capsule_pub_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    Image::ConstSharedPtr depth_image_;
    Image::ConstSharedPtr body_index_map_;
    CameraInfo::ConstSharedPtr camera_info_;

    std::map<int, std::vector<Point>> skeleton_points_;

    std::map<int, std::vector<vision::human::BodyJoint>>
        skeleton_joints_;

    std::map<int, builtin_interfaces::msg::Time> skeleton_stamps_;
    std::map<int, builtin_interfaces::msg::Time> zone_stamps_;

    std::set<int> active_body_ids_;
    std::set<int> published_voxel_ids_;

    std::set<std::pair<std::string, int>> published_capsule_ids_;

    rclcpp::Time last_depth_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_body_index_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_skeleton_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_zone_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_processed_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<ReconstructionNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("reconstruction_node"),
            "Reconstruction node stopped: %s",
            error.what());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}