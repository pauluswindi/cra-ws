#include "vision/workspace_monitor/workspace_monitor.hpp"
#include "vision/body_tracking/body_tracker.hpp"

namespace vision {
namespace workspace_monitor {

WorkspaceMonitor::WorkspaceMonitor(const WorkspaceBounds& bounds, double critical_shrink_factor)
: bounds_(bounds) {
    // Build smaller CRITICAL box centered inside the WARNING box.
    double cx = (bounds.min_x + bounds.max_x) / 2.0;
    double cy = (bounds.min_y + bounds.max_y) / 2.0;
    double cz = (bounds.min_z + bounds.max_z) / 2.0;
    double sx = (bounds.max_x - bounds.min_x) * critical_shrink_factor / 2.0;
    double sy = (bounds.max_y - bounds.min_y) * critical_shrink_factor / 2.0;
    double sz = (bounds.max_z - bounds.min_z) * critical_shrink_factor / 2.0;

    critical_bounds_ = {cx - sx, cx + sx, cy - sy, cy + sy, cz - sz, cz + sz};
}

bool WorkspaceMonitor::isInsideBounds(const geometry_msgs::msg::Point& point, const WorkspaceBounds& b) {
    return (point.x >= b.min_x && point.x <= b.max_x &&
            point.y >= b.min_y && point.y <= b.max_y &&
            point.z >= b.min_z && point.z <= b.max_z);
}

SafetyStatus WorkspaceMonitor::evaluate(const body_tracking::SkeletonData& skeleton) {
    if (!skeleton.is_valid) return SafetyStatus::SAFE;

    // Priority points for collision check: both hands and head.
    std::vector<geometry_msgs::msg::Point> priority_points;

    if (skeleton.is_tracked[body_tracking::JOINT_HAND_LEFT]) {
        priority_points.push_back(skeleton.joints[body_tracking::JOINT_HAND_LEFT]);
    }
    if (skeleton.is_tracked[body_tracking::JOINT_HAND_RIGHT]) {
        priority_points.push_back(skeleton.joints[body_tracking::JOINT_HAND_RIGHT]);
    }
    if (skeleton.is_tracked[body_tracking::JOINT_HEAD]) {
        priority_points.push_back(skeleton.joints[body_tracking::JOINT_HEAD]);
    }

    for (const auto& point : priority_points) {
        if (isInsideBounds(point, critical_bounds_)) return SafetyStatus::CRITICAL;
        if (isInsideBounds(point, bounds_)) return SafetyStatus::WARNING;
    }

    return SafetyStatus::SAFE;
}

std::string WorkspaceMonitor::statusToString(SafetyStatus status) {
    switch (status) {
        case SafetyStatus::CRITICAL: return "CRITICAL";
        case SafetyStatus::WARNING:  return "WARNING";
        default:                     return "SAFE";
    }
}

} // namespace workspace_monitor
} // namespace vision