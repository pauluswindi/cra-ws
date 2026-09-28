#include "vision/body_tracking/body_tracker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <map>

namespace vision {
namespace body_tracking {

BodyTracker::BodyTracker() {
    RCLCPP_INFO(rclcpp::get_logger("BodyTracker"), "Initialized (raw data, multi-person)");
}

int BodyTracker::getJointIndex(int marker_id) {
    int index = marker_id % 100;
    return (index >= 0 && index < 32) ? index : -1;
}

std::vector<SkeletonData> BodyTracker::parse(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    std::vector<SkeletonData> skeletons;
    if (!msg || msg->markers.empty()) {
        return skeletons;
    }

    std::map<int, SkeletonData> skeletons_by_id;

    for (const auto& marker : msg->markers) {
        const int body_id = marker.id / 100;
        const int joint_index = getJointIndex(marker.id);
        if (joint_index < 0) {
            continue;
        }

        SkeletonData& skeleton = skeletons_by_id[body_id];
        skeleton.body_id = body_id;
        skeleton.stamp = marker.header.stamp;
        skeleton.frame_id = marker.header.frame_id;
        skeleton.joints[joint_index] = marker.pose.position;
        skeleton.orientations[joint_index] = marker.pose.orientation;
        skeleton.is_tracked[joint_index] = true;
    }

    for (const auto& entry : skeletons_by_id) {
        skeletons.push_back(entry.second);
    }

    return skeletons;
}

} // namespace body_tracking
} // namespace vision