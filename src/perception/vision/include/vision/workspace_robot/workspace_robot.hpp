#ifndef VISION__WORKSPACE_ROBOT__WORKSPACE_ROBOT_HPP_
#define VISION__WORKSPACE_ROBOT__WORKSPACE_ROBOT_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

#include "vision/body_tracking/body_tracker.hpp"

namespace vision {
namespace workspace_robot {

enum class SafetyStatus { SAFE, WARNING, CRITICAL };

// Workspace footprint is an ordered polygon in the XY plane of base_link,
// closed automatically, any shape symmetric or not. Only x and y of each
// polygon point are used; the vertical extent comes from min_z and max_z.
// Robot link proximity is intentionally out of scope here: a future
// robot-side node will publish its link points and own the CRITICAL logic.
struct RobotWorkspace {
    std::vector<geometry_msgs::msg::Point> polygon;
    double min_z = 0.0;
    double max_z = 0.6;
};

class WorkspaceRobot {
public:
    explicit WorkspaceRobot(const RobotWorkspace& workspace);
    ~WorkspaceRobot() = default;

    SafetyStatus evaluate(const body_tracking::SkeletonData& skeleton) const;
    bool isPointInWorkspace(const geometry_msgs::msg::Point& p) const;
    std::string statusToString(SafetyStatus status) const;

private:
    bool insidePolygon(double x, double y) const;

    RobotWorkspace workspace_;
};

}  // namespace workspace_robot
}  // namespace vision

#endif  // VISION__WORKSPACE_ROBOT__WORKSPACE_ROBOT_HPP_