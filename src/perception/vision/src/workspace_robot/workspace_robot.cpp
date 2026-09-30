#include "vision/workspace_robot/workspace_robot.hpp"

namespace vision {
namespace workspace_robot {

WorkspaceRobot::WorkspaceRobot(const RobotWorkspace& workspace)
: workspace_(workspace) {}

bool WorkspaceRobot::insidePolygon(double x, double y) const {
    const auto& poly = workspace_.polygon;
    if (poly.size() < 3) {
        return false;
    }

    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const double xi = poly[i].x;
        const double yi = poly[i].y;
        const double xj = poly[j].x;
        const double yj = poly[j].y;

        const bool crosses = ((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (crosses) {
            inside = !inside;
        }
    }
    return inside;
}

bool WorkspaceRobot::isPointInWorkspace(const geometry_msgs::msg::Point& p) const {
    if (p.z < workspace_.min_z || p.z > workspace_.max_z) {
        return false;
    }
    return insidePolygon(p.x, p.y);
}

std::string WorkspaceRobot::statusToString(SafetyStatus status) const {
    switch (status) {
        case SafetyStatus::CRITICAL: return "CRITICAL";
        case SafetyStatus::WARNING: return "WARNING";
        default: return "SAFE";
    }
}

}  // namespace workspace_robot
}  // namespace vision