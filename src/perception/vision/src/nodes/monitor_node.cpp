#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "vision/body_tracking/body_tracker.hpp"
#include "vision/workspace_monitor/workspace_monitor.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace {
constexpr double kSkeletonTimeoutS = 1.0;
}

class MonitorNode : public rclcpp::Node {
public:
    MonitorNode() : Node("monitor_node") {
        vision::workspace_monitor::WorkspaceBounds bounds;
        bounds.min_x = declare_parameter("warn_min_x", 0.2);
        bounds.max_x = declare_parameter("warn_max_x", 0.8);
        bounds.min_y = declare_parameter("warn_min_y", -0.5);
        bounds.max_y = declare_parameter("warn_max_y", 0.5);
        bounds.min_z = declare_parameter("warn_min_z", 0.0);
        bounds.max_z = declare_parameter("warn_max_z", 0.6);
        const double shrink = declare_parameter("critical_shrink_factor", 0.6);

        body_tracker_ = std::make_unique<vision::body_tracking::BodyTracker>();
        workspace_monitor_ =
            std::make_unique<vision::workspace_monitor::WorkspaceMonitor>(bounds, shrink);

        skeleton_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "/vision/skeleton/upper_body_base", 10,
            std::bind(&MonitorNode::skeletonCallback, this, _1));

        status_pub_ = create_publisher<std_msgs::msg::String>("/robot/safety_status", 10);

        last_skeleton_time_s_ = now().seconds();
        watchdog_timer_ = create_wall_timer(
            200ms, std::bind(&MonitorNode::watchdogCallback, this));

        RCLCPP_INFO(get_logger(), "Monitor node started on /vision/skeleton/upper_body_base");
    }

private:
    void skeletonCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        last_skeleton_time_s_ = now().seconds();

        const auto skeletons = body_tracker_->parse(msg);

        using vision::workspace_monitor::SafetyStatus;
        SafetyStatus worst = SafetyStatus::SAFE;
        for (const auto& skeleton : skeletons) {
            const auto status = workspace_monitor_->evaluate(skeleton);
            if (status == SafetyStatus::CRITICAL) {
                worst = SafetyStatus::CRITICAL;
                break;
            }
            if (status == SafetyStatus::WARNING) {
                worst = SafetyStatus::WARNING;
            }
        }

        if (worst != last_status_) {
            last_status_ = worst;
            RCLCPP_INFO(get_logger(), "Safety status changed to: %s",
                        workspace_monitor_->statusToString(worst).c_str());
        }

        std_msgs::msg::String out;
        out.data = workspace_monitor_->statusToString(worst);
        status_pub_->publish(out);
    }

    void watchdogCallback() {
        const double elapsed = now().seconds() - last_skeleton_time_s_;
        if (elapsed <= kSkeletonTimeoutS) {
            return;
        }
        if (last_status_ != vision::workspace_monitor::SafetyStatus::CRITICAL) {
            last_status_ = vision::workspace_monitor::SafetyStatus::CRITICAL;
            RCLCPP_WARN(get_logger(),
                        "Skeleton data lost for %.2f s, publishing fail-safe CRITICAL", elapsed);
        }
        std_msgs::msg::String out;
        out.data = "CRITICAL";
        status_pub_->publish(out);
    }

    std::unique_ptr<vision::body_tracking::BodyTracker> body_tracker_;
    std::unique_ptr<vision::workspace_monitor::WorkspaceMonitor> workspace_monitor_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr skeleton_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    double last_skeleton_time_s_ = 0.0;
    vision::workspace_monitor::SafetyStatus last_status_ =
        vision::workspace_monitor::SafetyStatus::SAFE;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitorNode>());
    rclcpp::shutdown();
    return 0;
}