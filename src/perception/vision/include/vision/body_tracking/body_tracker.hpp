#ifndef VISION__BODY_TRACKING__BODY_TRACKER_HPP_
#define VISION__BODY_TRACKING__BODY_TRACKER_HPP_

#include <array>
#include <string>
#include <vector>
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace vision {
namespace body_tracking {

constexpr int JOINT_PELVIS = 0;
constexpr int JOINT_SPINE_NAVEL = 1;
constexpr int JOINT_SPINE_CHEST = 2;
constexpr int JOINT_NECK = 3;
constexpr int JOINT_CLAVICLE_LEFT = 4;
constexpr int JOINT_SHOULDER_LEFT = 5;
constexpr int JOINT_ELBOW_LEFT = 6;
constexpr int JOINT_WRIST_LEFT = 7;
constexpr int JOINT_HAND_LEFT = 8;
constexpr int JOINT_HANDTIP_LEFT = 9;
constexpr int JOINT_THUMB_LEFT = 10;
constexpr int JOINT_CLAVICLE_RIGHT = 11;
constexpr int JOINT_SHOULDER_RIGHT = 12;
constexpr int JOINT_ELBOW_RIGHT = 13;
constexpr int JOINT_WRIST_RIGHT = 14;
constexpr int JOINT_HAND_RIGHT = 15;
constexpr int JOINT_HANDTIP_RIGHT = 16;
constexpr int JOINT_THUMB_RIGHT = 17;
constexpr int JOINT_HEAD = 26;
constexpr int JOINT_NOSE = 27;
constexpr int JOINT_EYE_LEFT = 28;
constexpr int JOINT_EAR_LEFT = 29;
constexpr int JOINT_EYE_RIGHT = 30;
constexpr int JOINT_EAR_RIGHT = 31;

struct SkeletonData {
    int body_id = 0;
    std::array<geometry_msgs::msg::Point, 32> joints;
    std::array<geometry_msgs::msg::Quaternion, 32> orientations;
    std::array<bool, 32> is_tracked = {};
    bool is_valid = true;
    builtin_interfaces::msg::Time stamp;
    std::string frame_id;
};

class BodyTracker {
public:
    BodyTracker();
    ~BodyTracker() = default;

    static int getJointIndex(int marker_id);

    std::vector<SkeletonData> parse(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
};

} // namespace body_tracking
} // namespace vision

#endif // VISION__BODY_TRACKING__BODY_TRACKER_HPP_