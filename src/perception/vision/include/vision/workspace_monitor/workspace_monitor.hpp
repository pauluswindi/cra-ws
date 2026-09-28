#ifndef VISION__WORKSPACE_MONITOR__WORKSPACE_MONITOR_HPP_
#define VISION__WORKSPACE_MONITOR__WORKSPACE_MONITOR_HPP_

#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "vision/body_tracking/body_tracker.hpp"

namespace vision {
namespace workspace_monitor {

enum class SafetyStatus { SAFE, WARNING, CRITICAL };

struct WorkspaceBounds {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
};

class WorkspaceMonitor {
public:
    explicit WorkspaceMonitor(const WorkspaceBounds& bounds, double critical_shrink_factor = 0.5);
    SafetyStatus evaluate(const body_tracking::SkeletonData& skeleton);
    static std::string statusToString(SafetyStatus status);

private:
    bool isInsideBounds(const geometry_msgs::msg::Point& point, const WorkspaceBounds& bounds);
    WorkspaceBounds bounds_;
    WorkspaceBounds critical_bounds_; // Area lebih kecil untuk status CRITICAL
};

} // namespace workspace_monitor
} // namespace vision

#endif