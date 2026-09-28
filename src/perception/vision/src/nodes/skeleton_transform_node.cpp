#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vision/body_tracking/body_tracker.hpp"
#include "vision/frame/frame_transformer.hpp"

using namespace std::placeholders;

namespace {
constexpr char kBaseFrame[] = "base_link";

bool isUpperBodyJoint(int joint_index) {
    return joint_index <= 17 || (joint_index >= 26 && joint_index <= 31);
}
}  // namespace

class SkeletonTransformNode : public rclcpp::Node {
public:
    SkeletonTransformNode() : Node("skeleton_transform_node") {
        tf_transformer_ = std::make_unique<vision::frame::FrameTransformer>(this);

        skeleton_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "/body_tracking_data", 10,
            std::bind(&SkeletonTransformNode::skeletonCallback, this, _1));

        camera_frame_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/vision/skeleton/camera_frame", 10);
        base_frame_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/vision/skeleton/base_frame", 10);

        RCLCPP_INFO(get_logger(),
                    "Skeleton transform node started: upper body markers in camera and base frames");
    }

private:
    void skeletonCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        visualization_msgs::msg::MarkerArray filtered;
        for (const auto& marker : msg->markers) {
            const int joint_index = vision::body_tracking::BodyTracker::getJointIndex(marker.id);
            if (joint_index >= 0 && isUpperBodyJoint(joint_index)) {
                filtered.markers.push_back(marker);
            }
        }
        if (filtered.markers.empty()) {
            return;
        }

        camera_frame_pub_->publish(filtered);

        visualization_msgs::msg::MarkerArray base_markers;
        if (tf_transformer_->transformMarkers(filtered, base_markers, kBaseFrame)) {
            base_frame_pub_->publish(base_markers);
        }
    }

    std::unique_ptr<vision::frame::FrameTransformer> tf_transformer_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr skeleton_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr camera_frame_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr base_frame_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SkeletonTransformNode>());
    rclcpp::shutdown();
    return 0;
}