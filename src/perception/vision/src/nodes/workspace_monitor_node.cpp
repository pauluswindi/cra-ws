#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "vision/body_tracking/body_tracker.hpp"
#include "vision/workspace_robot/workspace_robot.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace {
constexpr double kSkeletonTimeoutS = 1.0;
constexpr char kZoneMarkerNs[] = "humans_in_zone";
}  // namespace

class WorkspaceMonitorNode : public rclcpp::Node {
public:
    WorkspaceMonitorNode() : Node("workspace_monitor_node") {
        vision::workspace_robot::RobotWorkspace ws;

        const auto poly = declare_parameter<std::vector<double>>(
            "workspace_polygon",
            std::vector<double>{0.2, -0.5, 0.8, -0.5, 0.9, 0.1, 0.6, 0.5, 0.2, 0.4});
        const auto z_range = declare_parameter<std::vector<double>>(
            "workspace_z", std::vector<double>{0.0, 0.6});

        if (poly.size() >= 6 && poly.size() % 2 == 0) {
            for (size_t i = 0; i + 1 < poly.size(); i += 2) {
                geometry_msgs::msg::Point corner;
                corner.x = poly[i];
                corner.y = poly[i + 1];
                ws.polygon.push_back(corner);
            }
        } else {
            RCLCPP_WARN(get_logger(),
                        "workspace_polygon needs at least 3 corners (6 values), zone disabled");
        }

        if (z_range.size() == 2) {
            ws.min_z = z_range[0];
            ws.max_z = z_range[1];
        } else {
            RCLCPP_WARN(get_logger(), "workspace_z needs exactly 2 values, using defaults");
        }

        workspace_def_ = ws;

        body_tracker_ = std::make_unique<vision::body_tracking::BodyTracker>();
        workspace_robot_ = std::make_unique<vision::workspace_robot::WorkspaceRobot>(ws);

        skeleton_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "/vision/skeleton/base_frame", 10,
            std::bind(&WorkspaceMonitorNode::skeletonCallback, this, _1));

        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/perception/workspace_status", 10);
        humans_in_zone_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/perception/humans_in_zone", 10);
        workspace_zone_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/perception/workspace_zone", rclcpp::QoS(1).transient_local());

        last_skeleton_time_s_ = now().seconds();
        watchdog_timer_ = create_wall_timer(
            200ms, std::bind(&WorkspaceMonitorNode::watchdogCallback, this));

        publishWorkspaceZone();

        RCLCPP_INFO(get_logger(),
                    "Workspace monitor started: %zu polygon corners, z [%.2f, %.2f]",
                    ws.polygon.size(), ws.min_z, ws.max_z);
    }

private:
    void skeletonCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        const double now_s = now().seconds();
        last_skeleton_time_s_ = now_s;

        const auto skeletons = body_tracker_->parse(msg);
        using SafetyStatus = vision::workspace_robot::SafetyStatus;

        visualization_msgs::msg::MarkerArray in_zone_markers;
        bool any_priority_in_zone = false;
        std::set<int> current_ids;

        for (const auto& skeleton : skeletons) {
            if (workspace_robot_->evaluate(skeleton) == SafetyStatus::WARNING) {
                any_priority_in_zone = true;
            }

            for (size_t idx = 0; idx < skeleton.joints.size(); ++idx) {
                if (!skeleton.is_tracked[idx]) {
                    continue;
                }
                const auto& point = skeleton.joints[idx];
                if (!workspace_robot_->isPointInWorkspace(point)) {
                    continue;
                }

                const int marker_id = skeleton.body_id * 100 + static_cast<int>(idx);
                current_ids.insert(marker_id);

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = skeleton.frame_id;
                marker.header.stamp = skeleton.stamp;
                marker.ns = kZoneMarkerNs;
                marker.id = marker_id;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position = point;
                marker.pose.orientation = skeleton.orientations[idx];
                marker.scale.x = 0.06;
                marker.scale.y = 0.06;
                marker.scale.z = 0.06;
                marker.color.r = 1.0;
                marker.color.g = 0.8;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                in_zone_markers.markers.push_back(marker);
            }
        }
        
        for (const int old_id : published_marker_ids_) {
            if (current_ids.count(old_id) == 0) {
                visualization_msgs::msg::Marker delete_marker;
                delete_marker.header.frame_id = "base_link";
                delete_marker.header.stamp = now();
                delete_marker.ns = kZoneMarkerNs;
                delete_marker.id = old_id;
                delete_marker.action = visualization_msgs::msg::Marker::DELETE;
                in_zone_markers.markers.push_back(delete_marker);
            }
        }
        published_marker_ids_ = current_ids;

        humans_in_zone_pub_->publish(in_zone_markers);

        const SafetyStatus worst =
            any_priority_in_zone ? SafetyStatus::WARNING : SafetyStatus::SAFE;
        if (worst != last_status_) {
            last_status_ = worst;
            RCLCPP_INFO(get_logger(), "Safety status changed to: %s",
                        workspace_robot_->statusToString(worst).c_str());
        }

        std_msgs::msg::String out;
        out.data = workspace_robot_->statusToString(worst);
        status_pub_->publish(out);
    }

    void publishWorkspaceZone() {
        const auto& poly = workspace_def_.polygon;
        if (poly.size() < 3) {
            return;
        }

        const double z_min = workspace_def_.min_z;
        const double z_max = workspace_def_.max_z;

        auto pointAt = [](const geometry_msgs::msg::Point& c, double z) {
            geometry_msgs::msg::Point p;
            p.x = c.x;
            p.y = c.y;
            p.z = z;
            return p;
        };

        visualization_msgs::msg::Marker edges;
        edges.header.frame_id = "base_link";
        edges.header.stamp = now();
        edges.ns = "workspace_zone";
        edges.id = 0;
        edges.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges.action = visualization_msgs::msg::Marker::ADD;
        edges.pose.orientation.w = 1.0;
        edges.scale.x = 0.02;
        edges.color.r = 1.0;
        edges.color.g = 0.8;
        edges.color.b = 0.0;
        edges.color.a = 0.9;

        visualization_msgs::msg::Marker floor_fill;
        floor_fill.header = edges.header;
        floor_fill.ns = "workspace_zone";
        floor_fill.id = 1;
        floor_fill.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        floor_fill.action = visualization_msgs::msg::Marker::ADD;
        floor_fill.pose.orientation.w = 1.0;
        floor_fill.color.r = 1.0;
        floor_fill.color.g = 0.8;
        floor_fill.color.b = 0.0;
        floor_fill.color.a = 0.15;

        for (size_t i = 0; i < poly.size(); ++i) {
            const size_t j = (i + 1) % poly.size();
            edges.points.push_back(pointAt(poly[i], z_min));
            edges.points.push_back(pointAt(poly[j], z_min));
            edges.points.push_back(pointAt(poly[i], z_max));
            edges.points.push_back(pointAt(poly[j], z_max));
            edges.points.push_back(pointAt(poly[i], z_min));
            edges.points.push_back(pointAt(poly[i], z_max));

            if (i >= 1 && i + 1 < poly.size()) {
                floor_fill.points.push_back(pointAt(poly[0], z_min));
                floor_fill.points.push_back(pointAt(poly[i], z_min));
                floor_fill.points.push_back(pointAt(poly[i + 1], z_min));
            }
        }

        visualization_msgs::msg::MarkerArray markers;
        markers.markers.push_back(edges);
        markers.markers.push_back(floor_fill);
        workspace_zone_pub_->publish(markers);
    }

    void watchdogCallback() {
        const double elapsed = now().seconds() - last_skeleton_time_s_;
        if (elapsed <= kSkeletonTimeoutS) {
            return;
        }

        if (last_status_ != vision::workspace_robot::SafetyStatus::CRITICAL) {
            last_status_ = vision::workspace_robot::SafetyStatus::CRITICAL;
            RCLCPP_WARN(get_logger(),
                        "Skeleton data lost for %.2f s, publishing fail-safe CRITICAL", elapsed);

            visualization_msgs::msg::MarkerArray clear_msg;
            for (const int old_id : published_marker_ids_) {
                visualization_msgs::msg::Marker delete_marker;
                delete_marker.ns = kZoneMarkerNs;
                delete_marker.id = old_id;
                delete_marker.action = visualization_msgs::msg::Marker::DELETE;
                clear_msg.markers.push_back(delete_marker);
            }
            published_marker_ids_.clear();
            humans_in_zone_pub_->publish(clear_msg);
        }

        std_msgs::msg::String out;
        out.data = "CRITICAL";
        status_pub_->publish(out);
    }

    std::unique_ptr<vision::body_tracking::BodyTracker> body_tracker_;
    std::unique_ptr<vision::workspace_robot::WorkspaceRobot> workspace_robot_;
    vision::workspace_robot::RobotWorkspace workspace_def_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr skeleton_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr humans_in_zone_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr workspace_zone_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    std::set<int> published_marker_ids_;
    double last_skeleton_time_s_ = 0.0;
    vision::workspace_robot::SafetyStatus last_status_ =
        vision::workspace_robot::SafetyStatus::SAFE;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorkspaceMonitorNode>());
    rclcpp::shutdown();
    return 0;
}