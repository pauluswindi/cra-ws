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
#include "msgs/msg/human_capsule_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "vision/human/capsule_geometry.hpp"
#include "vision/human/capsule_marker.hpp"
#include "vision/reconstruction/reconstruction_pipeline.hpp"
#include "vision/reconstruction/voxel_marker.hpp"

namespace
{

constexpr char kBaseFrame[] = "base_link";
constexpr char kZoneNamespace[] = "humans_in_zone";
constexpr char kVoxelNamespace[] = "human_voxels";

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Point = geometry_msgs::msg::Point;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using HumanCapsuleArray = msgs::msg::HumanCapsuleArray;

using MarkerIds = vision::human::MarkerIds;
using SkeletonData = vision::reconstruction::SkeletonData;
using ReconstructionConfig = vision::reconstruction::ReconstructionConfig;
using ReconstructionResult = vision::reconstruction::ReconstructionResult;
using ReconstructionPipeline = vision::reconstruction::ReconstructionPipeline;

bool isUpperBodyJoint(int id)
{
    return (id >= 0 && id <= 17) ||
           (id >= 26 && id <= 31);
}

bool isFinite(const Point& point)
{
    return std::isfinite(point.x) &&
           std::isfinite(point.y) &&
           std::isfinite(point.z);
}

double stampDifference(
    const builtin_interfaces::msg::Time& a,
    const builtin_interfaces::msg::Time& b)
{
    return std::abs(
        (rclcpp::Time(a) - rclcpp::Time(b)).seconds());
}

void appendDeleteMarkers(
    const MarkerIds& previous,
    const MarkerIds& current,
    const builtin_interfaces::msg::Time& stamp,
    MarkerArray& output)
{
    for (const auto& id : previous) {
        if (current.count(id) != 0) {
            continue;
        }

        Marker marker;
        marker.header.frame_id = kBaseFrame;
        marker.header.stamp = stamp;
        marker.ns = id.first;
        marker.id = id.second;
        marker.action = Marker::DELETE;

        output.markers.push_back(std::move(marker));
    }
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
        loadParameters();

        pipeline_ = std::make_unique<ReconstructionPipeline>(
            reconstruction_config_);

        const auto sensor_qos = rclcpp::SensorDataQoS();

        depth_sub_ = create_subscription<Image>(
            depth_topic_,
            sensor_qos,
            [this](Image::ConstSharedPtr msg) {
                depth_image_ = std::move(msg);
                last_depth_received_ = now();
                processFrame();
            });

        body_index_sub_ = create_subscription<Image>(
            body_index_topic_,
            sensor_qos,
            [this](Image::ConstSharedPtr msg) {
                body_index_map_ = std::move(msg);
                last_body_index_received_ = now();
                processFrame();
            });

        camera_info_sub_ = create_subscription<CameraInfo>(
            camera_info_topic_,
            sensor_qos,
            [this](CameraInfo::ConstSharedPtr msg) {
                camera_info_ = std::move(msg);
            });

        skeleton_sub_ = create_subscription<MarkerArray>(
            "/vision/skeleton/base_frame",
            10,
            [this](MarkerArray::ConstSharedPtr msg) {
                skeletonCallback(*msg);
            });

        zone_sub_ = create_subscription<MarkerArray>(
            "/perception/humans_in_zone",
            10,
            [this](MarkerArray::ConstSharedPtr msg) {
                zoneCallback(*msg);
            });

        capsule_pub_ = create_publisher<MarkerArray>(
            "/vision/human/capsules",
            10);

        geometry_pub_ = create_publisher<HumanCapsuleArray>(
            "/vision/human/geometry",
            rclcpp::QoS(10).reliable());

        if (publish_voxels_) {
            voxel_pub_ = create_publisher<MarkerArray>(
                "/vision/reconstruction/voxels",
                10);
        }

        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                watchdogCallback();
            });

        publishInvalidGeometry();

        RCLCPP_INFO(
            get_logger(),
            "Reconstruction node started "
            "(geometry: enabled, voxel debug: %s)",
            publish_voxels_ ? "enabled" : "disabled");
    }

private:
    void loadParameters()
    {
        depth_topic_ = declare_parameter<std::string>(
            "depth_topic", "/depth/image_raw");

        camera_info_topic_ = declare_parameter<std::string>(
            "camera_info_topic", "/depth/camera_info");

        body_index_topic_ = declare_parameter<std::string>(
            "body_index_topic", "/body_index_map/image_raw");

        auto& config = reconstruction_config_;

        config.roi_margin = declare_parameter<double>(
            "roi_margin", 0.15);

        config.voxel_size = declare_parameter<double>(
            "voxel_size", 0.02);

        config.min_depth = declare_parameter<double>(
            "min_depth", 0.3);

        config.max_depth = declare_parameter<double>(
            "max_depth", 3.0);

        max_sync_delay_ = declare_parameter<double>(
            "max_sync_delay", 0.15);

        data_timeout_ = declare_parameter<double>(
            "data_timeout", 0.5);

        publish_voxels_ = declare_parameter<bool>(
            "publish_voxels", false);

        auto& capsule = config.capsule;

        capsule.torso_radius = declare_parameter<double>(
            "capsule.torso_radius", 0.13);

        capsule.head_radius = declare_parameter<double>(
            "capsule.head_radius", 0.09);

        capsule.upper_arm_radius = declare_parameter<double>(
            "capsule.upper_arm_radius", 0.055);

        capsule.forearm_radius = declare_parameter<double>(
            "capsule.forearm_radius", 0.045);

        capsule.hand_radius = declare_parameter<double>(
            "capsule.hand_radius", 0.05);

        capsule.torso_max_radius = declare_parameter<double>(
            "capsule.torso_max_radius", 0.22);

        capsule.head_max_radius = declare_parameter<double>(
            "capsule.head_max_radius", 0.14);

        capsule.upper_arm_max_radius = declare_parameter<double>(
            "capsule.upper_arm_max_radius", 0.10);

        capsule.forearm_max_radius = declare_parameter<double>(
            "capsule.forearm_max_radius", 0.085);

        capsule.hand_max_radius = declare_parameter<double>(
            "capsule.hand_max_radius", 0.09);

        capsule.safety_margin = declare_parameter<double>(
            "capsule.safety_margin", 0.015);

        capsule.radius_percentile = declare_parameter<double>(
            "capsule.radius_percentile", 0.85);

        const int min_voxels = declare_parameter<int>(
            "capsule.min_voxels", 10);

        config.radius_smoothing_alpha =
            declare_parameter<double>(
                "capsule.radius_smoothing_alpha", 0.2);

        if (min_voxels < 1 ||
            !std::isfinite(max_sync_delay_) ||
            max_sync_delay_ < 0.0 ||
            !std::isfinite(data_timeout_) ||
            data_timeout_ <= 0.0) {
            throw std::invalid_argument(
                "Invalid reconstruction node configuration");
        }

        capsule.min_voxels =
            static_cast<std::size_t>(min_voxels);
    }

    void skeletonCallback(const MarkerArray& msg)
    {
        skeletons_.clear();
        skeleton_stamps_.clear();

        for (const auto& marker : msg.markers) {
            if (marker.action != Marker::ADD ||
                marker.id < 0 ||
                marker.header.frame_id != kBaseFrame ||
                !isFinite(marker.pose.position)) {
                continue;
            }

            const int body_id = marker.id / 100;
            const int joint_id = marker.id % 100;

            if (!isUpperBodyJoint(joint_id)) {
                continue;
            }

            auto& skeleton = skeletons_[body_id];

            skeleton.points.push_back(marker.pose.position);

            vision::human::BodyJoint joint;
            joint.id = joint_id;
            joint.position = marker.pose.position;

            skeleton.joints.push_back(joint);
            skeleton_stamps_[body_id] = marker.header.stamp;
        }

        last_skeleton_received_ = now();
        processFrame();
    }

    void zoneCallback(const MarkerArray& msg)
    {
        std::set<int> current_ids;

        for (const auto& marker : msg.markers) {
            if (marker.ns != kZoneNamespace ||
                marker.action != Marker::ADD ||
                marker.id < 0 ||
                marker.header.frame_id != kBaseFrame) {
                continue;
            }

            current_ids.insert(marker.id / 100);
        }

        active_body_ids_ = std::move(current_ids);
        last_zone_received_ = now();

        if (active_body_ids_.empty()) {
            clearMarkers();

            if (!publishEmptyGeometry()) {
                publishInvalidGeometry();
            }

            return;
        }

        processFrame();
    }

    bool isRecent(const rclcpp::Time& stamp) const
    {
        if (stamp.nanoseconds() == 0) {
            return false;
        }

        const double age = (now() - stamp).seconds();

        return age >= 0.0 && age <= data_timeout_;
    }

    bool basicInputsReady() const
    {
        return depth_image_ &&
               body_index_map_ &&
               camera_info_ &&
               isRecent(last_depth_received_) &&
               isRecent(last_body_index_received_) &&
               isRecent(last_zone_received_);
    }

    bool inputsReady() const
    {
        return basicInputsReady() &&
               !active_body_ids_.empty() &&
               isRecent(last_skeleton_received_);
    }

    bool basicFrameValid()
    {
        const auto& depth = *depth_image_;
        const auto& body_map = *body_index_map_;
        const auto& info = *camera_info_;

        if (stampDifference(
                depth.header.stamp,
                body_map.header.stamp) > max_sync_delay_) {
            return false;
        }

        if (depth.width != body_map.width ||
            depth.height != body_map.height ||
            depth.width != info.width ||
            depth.height != info.height ||
            depth.header.frame_id != body_map.header.frame_id ||
            depth.header.frame_id != info.header.frame_id) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Depth, body index, or camera info mismatch");

            return false;
        }

        return true;
    }

    bool frameValid()
    {
        if (!basicFrameValid()) {
            return false;
        }

        const auto& depth = *depth_image_;
        std::set<std::uint8_t> body_indices;

        for (const int body_id : active_body_ids_) {
            const auto index =
                static_cast<std::uint8_t>(body_id % 255);

            if (!body_indices.insert(index).second) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Ambiguous body index detected");

                return false;
            }

            const auto skeleton_it = skeletons_.find(body_id);
            const auto stamp_it = skeleton_stamps_.find(body_id);

            if (skeleton_it == skeletons_.end() ||
                skeleton_it->second.joints.empty() ||
                stamp_it == skeleton_stamps_.end() ||
                stampDifference(
                    depth.header.stamp,
                    stamp_it->second) > max_sync_delay_) {
                return false;
            }
        }

        return true;
    }

    void publishInvalidGeometry()
    {
        if (invalid_geometry_published_) {
            return;
        }

        HumanCapsuleArray output;
        output.header.frame_id = kBaseFrame;
        output.header.stamp = now();
        output.valid = false;

        geometry_pub_->publish(output);
        invalid_geometry_published_ = true;
    }

    bool publishEmptyGeometry()
    {
        if (!active_body_ids_.empty() ||
            !basicInputsReady() ||
            !basicFrameValid()) {
            return false;
        }

        const rclcpp::Time frame_stamp(
            depth_image_->header.stamp);

        if (frame_stamp == last_empty_stamp_ &&
            !invalid_geometry_published_) {
            return true;
        }

        HumanCapsuleArray output;
        output.header.frame_id = kBaseFrame;
        output.header.stamp = depth_image_->header.stamp;
        output.valid = true;

        geometry_pub_->publish(output);

        invalid_geometry_published_ = false;
        last_empty_stamp_ = frame_stamp;

        return true;
    }

    void processFrame()
    {
        if (active_body_ids_.empty()) {
            publishEmptyGeometry();
            return;
        }

        if (!inputsReady() || !frameValid()) {
            return;
        }

        const auto& depth = *depth_image_;

        const rclcpp::Time frame_stamp(depth.header.stamp);

        if (frame_stamp == last_processed_stamp_) {
            return;
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
                "Depth transform unavailable: %s",
                error.what());

            return;
        }

        ReconstructionResult result;

        try {
            if (!pipeline_->process(
                    depth,
                    *body_index_map_,
                    *camera_info_,
                    transform,
                    skeletons_,
                    active_body_ids_,
                    publish_voxels_,
                    result)) {
                return;
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Human reconstruction failed: %s",
                error.what());

            return;
        }

        MarkerArray capsule_output;
        MarkerArray voxel_output;

        MarkerIds current_capsule_ids;
        MarkerIds current_voxel_ids;

        std::vector<vision::human::Capsule> all_capsules;
        all_capsules.reserve(result.bodies.size() * 8);

        for (const auto& body : result.bodies) {
            all_capsules.insert(
                all_capsules.end(),
                body.capsules.begin(),
                body.capsules.end());

            vision::human::appendCapsuleMarkers(
                body.capsules,
                depth.header.stamp,
                capsule_output,
                current_capsule_ids);

            if (publish_voxels_) {
                const auto old_size = voxel_output.markers.size();

                vision::reconstruction::appendVoxelMarker(
                    body.voxels,
                    body.body_id,
                    reconstruction_config_.voxel_size,
                    depth.header.stamp,
                    voxel_output);

                if (voxel_output.markers.size() > old_size) {
                    current_voxel_ids.emplace(
                        kVoxelNamespace,
                        static_cast<int>(body.body_id));
                }

                RCLCPP_DEBUG_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Body %u: voxel coverage %.1f%% (%zu/%zu)",
                    body.body_id,
                    100.0 * body.coverage.ratio(),
                    body.coverage.covered_voxels,
                    body.coverage.total_voxels);
            }
        }

        if (all_capsules.empty()) {
            return;
        }

        auto geometry_output =
            vision::human::makeGeometryMessage(
                all_capsules,
                depth.header.stamp);

        appendDeleteMarkers(
            published_capsule_ids_,
            current_capsule_ids,
            depth.header.stamp,
            capsule_output);

        if (publish_voxels_) {
            appendDeleteMarkers(
                published_voxel_ids_,
                current_voxel_ids,
                depth.header.stamp,
                voxel_output);
        }

        published_capsule_ids_ =
            std::move(current_capsule_ids);

        if (publish_voxels_) {
            published_voxel_ids_ =
                std::move(current_voxel_ids);
        }

        last_processed_stamp_ = frame_stamp;
        last_successful_update_ = now();

        geometry_pub_->publish(geometry_output);
        invalid_geometry_published_ = false;

        if (!capsule_output.markers.empty()) {
            capsule_pub_->publish(capsule_output);
        }

        if (publish_voxels_ &&
            !voxel_output.markers.empty()) {
            voxel_pub_->publish(voxel_output);
        }
    }

    void clearMarkers()
    {
        const builtin_interfaces::msg::Time stamp = now();

        if (!published_capsule_ids_.empty()) {
            MarkerArray output;

            appendDeleteMarkers(
                published_capsule_ids_,
                MarkerIds{},
                stamp,
                output);

            capsule_pub_->publish(output);
            published_capsule_ids_.clear();
        }

        if (publish_voxels_ &&
            !published_voxel_ids_.empty()) {
            MarkerArray output;

            appendDeleteMarkers(
                published_voxel_ids_,
                MarkerIds{},
                stamp,
                output);

            voxel_pub_->publish(output);
            published_voxel_ids_.clear();
        }

        pipeline_->reset();
    }

    void clearOutputs()
    {
        clearMarkers();
        publishInvalidGeometry();
    }

    void watchdogCallback()
    {
        if (active_body_ids_.empty()) {
            if (!publishEmptyGeometry()) {
                clearOutputs();
            }

            return;
        }

        if (!inputsReady() ||
            !isRecent(last_successful_update_)) {
            clearOutputs();
        }
    }

    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string body_index_topic_;

    double max_sync_delay_ = 0.15;
    double data_timeout_ = 0.5;

    bool publish_voxels_ = false;
    bool invalid_geometry_published_ = false;

    ReconstructionConfig reconstruction_config_;

    std::unique_ptr<ReconstructionPipeline> pipeline_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<Image>::SharedPtr body_index_sub_;
    rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Subscription<MarkerArray>::SharedPtr skeleton_sub_;
    rclcpp::Subscription<MarkerArray>::SharedPtr zone_sub_;

    rclcpp::Publisher<MarkerArray>::SharedPtr capsule_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr voxel_pub_;

    rclcpp::Publisher<HumanCapsuleArray>::SharedPtr geometry_pub_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    Image::ConstSharedPtr depth_image_;
    Image::ConstSharedPtr body_index_map_;
    CameraInfo::ConstSharedPtr camera_info_;

    std::map<int, SkeletonData> skeletons_;

    std::map<int, builtin_interfaces::msg::Time> skeleton_stamps_;

    std::set<int> active_body_ids_;

    MarkerIds published_capsule_ids_;
    MarkerIds published_voxel_ids_;

    rclcpp::Time last_depth_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_body_index_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_skeleton_received_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_zone_received_{0, 0, RCL_ROS_TIME};

    rclcpp::Time last_processed_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_empty_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_successful_update_{0, 0, RCL_ROS_TIME};
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