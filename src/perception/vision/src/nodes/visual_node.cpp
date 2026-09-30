#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "vision/frame/frame_transformer.hpp"

using namespace std::chrono_literals;

namespace {

constexpr int ID_PELVIS = 100;
constexpr int ID_SPINE_NAVEL = 101;
constexpr int ID_SPINE_CHEST = 102;
constexpr int ID_NECK = 103;
constexpr int ID_CLAVICLE_LEFT = 104;
constexpr int ID_SHOULDER_LEFT = 105;
constexpr int ID_ELBOW_LEFT = 106;
constexpr int ID_WRIST_LEFT = 107;
constexpr int ID_HAND_LEFT = 108;
constexpr int ID_HANDTIP_LEFT = 109;
constexpr int ID_THUMB_LEFT = 110;
constexpr int ID_CLAVICLE_RIGHT = 111;
constexpr int ID_SHOULDER_RIGHT = 112;
constexpr int ID_ELBOW_RIGHT = 113;
constexpr int ID_WRIST_RIGHT = 114;
constexpr int ID_HAND_RIGHT = 115;
constexpr int ID_HANDTIP_RIGHT = 116;
constexpr int ID_THUMB_RIGHT = 117;
constexpr int ID_HEAD = 126;
constexpr int ID_NOSE = 127;
constexpr int ID_EYE_LEFT = 128;
constexpr int ID_EAR_LEFT = 129;
constexpr int ID_EYE_RIGHT = 130;
constexpr int ID_EAR_RIGHT = 131;

constexpr double kMinValidDepthM = 0.1;
constexpr double kBaseDataTimeoutS = 0.5;
constexpr double kImageTimeoutS = 1.0;
constexpr double kStaleDataTimeoutS = 0.5;
// How long the raw topic must stay silent before the compressed topic
// takes over. The fork driver publishes color either as raw Image (bgra)
// or as CompressedImage (jpeg), never both, so one second is plenty.
constexpr double kRawSourceTimeoutS = 1.0;

constexpr double kDefaultDisplayFps = 30.0;
constexpr double kMinDisplayFps = 1.0;
constexpr double kMaxDisplayFps = 60.0;

const std::vector<std::pair<int, int>> kBones = {
    {ID_NECK, ID_SPINE_CHEST},
    {ID_SPINE_CHEST, ID_SPINE_NAVEL},
    {ID_SPINE_NAVEL, ID_PELVIS},
    {ID_HEAD, ID_NECK},
    {ID_NOSE, ID_HEAD},
    {ID_SPINE_CHEST, ID_CLAVICLE_LEFT},
    {ID_CLAVICLE_LEFT, ID_SHOULDER_LEFT},
    {ID_SHOULDER_LEFT, ID_ELBOW_LEFT},
    {ID_ELBOW_LEFT, ID_WRIST_LEFT},
    {ID_WRIST_LEFT, ID_HAND_LEFT},
    {ID_HAND_LEFT, ID_HANDTIP_LEFT},
    {ID_HAND_LEFT, ID_THUMB_LEFT},
    {ID_SPINE_CHEST, ID_CLAVICLE_RIGHT},
    {ID_CLAVICLE_RIGHT, ID_SHOULDER_RIGHT},
    {ID_SHOULDER_RIGHT, ID_ELBOW_RIGHT},
    {ID_ELBOW_RIGHT, ID_WRIST_RIGHT},
    {ID_WRIST_RIGHT, ID_HAND_RIGHT},
    {ID_HAND_RIGHT, ID_HANDTIP_RIGHT},
    {ID_HAND_RIGHT, ID_THUMB_RIGHT},
};

const std::vector<cv::Scalar> kPersonColors = {
    cv::Scalar(0, 255, 0),
    cv::Scalar(0, 255, 255),
    cv::Scalar(255, 0, 255),
    cv::Scalar(255, 255, 0),
    cv::Scalar(0, 165, 255),
    cv::Scalar(255, 255, 255),
};

cv::Scalar personColor(int body_id) {
    return kPersonColors[static_cast<size_t>(body_id) % kPersonColors.size()];
}

std::map<int, cv::Scalar> getJointColors() {
    return {
        {ID_PELVIS, cv::Scalar(200, 200, 0)}, {ID_SPINE_NAVEL, cv::Scalar(200, 150, 0)},
        {ID_SPINE_CHEST, cv::Scalar(150, 200, 0)}, {ID_NECK, cv::Scalar(100, 200, 100)},
        {ID_HEAD, cv::Scalar(100, 150, 200)}, {ID_NOSE, cv::Scalar(50, 100, 200)},
        {ID_EYE_LEFT, cv::Scalar(100, 50, 200)}, {ID_EYE_RIGHT, cv::Scalar(150, 50, 200)},
        {ID_EAR_LEFT, cv::Scalar(200, 50, 150)}, {ID_EAR_RIGHT, cv::Scalar(200, 50, 100)},
        {ID_CLAVICLE_LEFT, cv::Scalar(200, 100, 100)}, {ID_CLAVICLE_RIGHT, cv::Scalar(200, 100, 150)},
        {ID_SHOULDER_LEFT, cv::Scalar(150, 100, 200)}, {ID_SHOULDER_RIGHT, cv::Scalar(100, 100, 200)},
        {ID_ELBOW_LEFT, cv::Scalar(100, 150, 150)}, {ID_ELBOW_RIGHT, cv::Scalar(100, 200, 150)},
        {ID_WRIST_LEFT, cv::Scalar(100, 200, 100)}, {ID_WRIST_RIGHT, cv::Scalar(150, 200, 100)},
        {ID_HAND_LEFT, cv::Scalar(200, 200, 100)}, {ID_HAND_RIGHT, cv::Scalar(200, 150, 100)},
        {ID_HANDTIP_LEFT, cv::Scalar(200, 100, 50)}, {ID_HANDTIP_RIGHT, cv::Scalar(200, 50, 50)},
        {ID_THUMB_LEFT, cv::Scalar(150, 50, 50)}, {ID_THUMB_RIGHT, cv::Scalar(100, 50, 100)},
    };
}

const char* jointName(int marker_id) {
    switch (marker_id) {
        case ID_HAND_LEFT: return "L-Hand";
        case ID_HAND_RIGHT: return "R-Hand";
        case ID_HEAD: return "Head";
        default: return "Joint";
    }
}

}  // namespace

class VisualNode : public rclcpp::Node {
public:
    VisualNode() : Node("visual_node") {
        tf_transformer_ = std::make_unique<vision::frame::FrameTransformer>(this);

        // Display FPS parameter (must be passed as double in launch, e.g., 20.0)
        double display_fps = declare_parameter("display_target_fps", kDefaultDisplayFps);
        if (display_fps < kMinDisplayFps || display_fps > kMaxDisplayFps) {
            RCLCPP_WARN(get_logger(),
                        "display_target_fps %.1f out of range [%.0f, %.0f], using %.0f",
                        display_fps, kMinDisplayFps, kMaxDisplayFps, kDefaultDisplayFps);
            display_fps = kDefaultDisplayFps;
        }
        const auto display_period =
            std::chrono::milliseconds(static_cast<int64_t>(1000.0 / display_fps));

        // Display scale parameter (0.5 = half resolution, 0.25 = quarter resolution)
        display_scale_ = declare_parameter("display_scale", 1.0);
        if (display_scale_ <= 0.0 || display_scale_ > 1.0) {
            RCLCPP_WARN(get_logger(), "display_scale %.2f invalid, using 1.0", display_scale_);
            display_scale_ = 1.0;
        }

        auto latest_only = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/rgb/image_raw", latest_only,
            std::bind(&VisualNode::imageCallback, this, std::placeholders::_1));

        // Fallback color source: carries the MJPG bytes when the driver runs
        // in jpeg mode and leaves /rgb/image_raw silent.
        compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/rgb/image_raw/compressed", latest_only,
            std::bind(&VisualNode::compressedImageCallback, this, std::placeholders::_1));

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/rgb/camera_info", latest_only,
            std::bind(&VisualNode::cameraInfoCallback, this, std::placeholders::_1));

        camera_skeleton_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "/vision/skeleton/camera_frame", latest_only,
            std::bind(&VisualNode::cameraSkeletonCallback, this, std::placeholders::_1));

        base_skeleton_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "/vision/skeleton/base_frame", latest_only,
            std::bind(&VisualNode::baseSkeletonCallback, this, std::placeholders::_1));

        zone_skeleton_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "/perception/humans_in_zone", latest_only,
            std::bind(&VisualNode::zoneSkeletonCallback, this, std::placeholders::_1));

        status_sub_ = create_subscription<std_msgs::msg::String>(
            "/perception/workspace_status", 10,
            std::bind(&VisualNode::statusCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(display_period,
                                   std::bind(&VisualNode::displayCallback, this));

        joint_colors_ = getJointColors();
        RCLCPP_INFO(get_logger(),
                    "Visual node started (FPV view, multi-person, workspace-filtered, "
                    "display %.0f fps, scale %.2f, auto color source)",
                    display_fps, display_scale_);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        const double now_s = now().seconds();

        cv::Mat frame;
        try {
            if (msg->encoding == "jpeg" || msg->encoding == "mjpeg" ||
                msg->encoding == "mjpg" || msg->encoding == "jpg") {
                const cv::Mat buf(1, static_cast<int>(msg->data.size()), CV_8UC1,
                                  const_cast<uint8_t*>(msg->data.data()));
                frame = cv::imdecode(buf, cv::IMREAD_COLOR);
                if (frame.empty()) {
                    RCLCPP_ERROR(get_logger(), "Failed to decode jpeg color image");
                    return;
                }
            } else {
                frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            }
            if (display_scale_ < 1.0 && !frame.empty()) {
                const int target_w = static_cast<int>(frame.cols * display_scale_);
                const int target_h = static_cast<int>(frame.rows * display_scale_);
                cv::resize(frame, frame, cv::Size(target_w, target_h));
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        last_raw_msg_time_s_ = now_s;
        if (using_compressed_) {
            RCLCPP_INFO(get_logger(), "Color source switched back to /rgb/image_raw");
            using_compressed_ = false;
        }
        current_image_ = frame;
        has_image_ = true;
        last_image_size_ = frame.size();
        if (last_image_time_s_ > 0.0) {
            const double dt = now_s - last_image_time_s_;
            if (dt > 1e-6) {
                fps_ = 0.9 * fps_ + 0.1 / dt;
            }
        }
        last_image_time_s_ = now_s;
    }

    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        // Automatic source switch: accept compressed frames only while the raw
        // topic has been silent, so a stray compressed duplicate in bgra mode
        // is ignored with a single time comparison and no decode cost.
        const double now_s = now().seconds();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if ((now_s - last_raw_msg_time_s_) < kRawSourceTimeoutS) {
                return;
            }
        }

        cv::Mat frame;
        try {
            const cv::Mat buf(1, static_cast<int>(msg->data.size()), CV_8UC1,
                              const_cast<uint8_t*>(msg->data.data()));
            frame = cv::imdecode(buf, cv::IMREAD_COLOR);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(get_logger(), "compressed decode exception: %s", e.what());
            return;
        }
        if (frame.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to decode compressed color image");
            return;
        }
        if (display_scale_ < 1.0) {
            const int target_w = static_cast<int>(frame.cols * display_scale_);
            const int target_h = static_cast<int>(frame.rows * display_scale_);
            cv::resize(frame, frame, cv::Size(target_w, target_h));
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!using_compressed_) {
            RCLCPP_INFO(get_logger(), "Color source switched to /rgb/image_raw/compressed");
            using_compressed_ = true;
        }
        current_image_ = frame;
        has_image_ = true;
        last_image_size_ = frame.size();
        if (last_image_time_s_ > 0.0) {
            const double dt = now_s - last_image_time_s_;
            if (dt > 1e-6) {
                fps_ = 0.9 * fps_ + 0.1 / dt;
            }
        }
        last_image_time_s_ = now_s;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        camera_info_ = *msg;
        has_camera_info_ = true;
    }

    void cameraSkeletonCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_camera_skeleton_time_s_ = now().seconds();
        camera_people_.clear();
        projected_people_.clear();
        for (const auto& marker : msg->markers) {
            camera_people_[marker.id / 100][marker.id] = marker.pose.position;
            camera_frame_id_ = marker.header.frame_id;
        }

        if (!has_camera_info_ || camera_frame_id_.empty()) {
            return;
        }

        const double fx = camera_info_.k[0];
        const double fy = camera_info_.k[4];
        const double cx = camera_info_.k[2];
        const double cy = camera_info_.k[5];
        const std::string target_frame = camera_info_.header.frame_id;

        for (const auto& person : camera_people_) {
            for (const auto& joint : person.second) {
                const geometry_msgs::msg::Point p = tf_transformer_->transformPoint(
                    joint.second, target_frame, camera_frame_id_);
                if (p.z <= kMinValidDepthM) {
                    projected_people_[person.first][joint.first] = cv::Point(-1, -1);
                    continue;
                }
                // Scale projection to match downscaled display
                projected_people_[person.first][joint.first] =
                    cv::Point(static_cast<int>((p.x * fx / p.z + cx) * display_scale_),
                              static_cast<int>((p.y * fy / p.z + cy) * display_scale_));
            }
        }
    }

    void baseSkeletonCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        base_people_.clear();
        for (const auto& marker : msg->markers) {
            base_people_[marker.id / 100][marker.id] = marker.pose.position;
        }
        last_base_time_s_ = now().seconds();
    }

    void zoneSkeletonCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_body_ids_.clear();
        for (const auto& marker : msg->markers) {
            if (marker.action != visualization_msgs::msg::Marker::ADD) {
                continue;
            }
            active_body_ids_.insert(marker.id / 100);
        }
        last_zone_time_s_ = now().seconds();
    }

    void statusCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        safety_status_ = msg->data;
    }

    void displayCallback() {
        cv::Mat display;
        bool signal_lost = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (has_image_ && !current_image_.empty()) {
                std::swap(display, current_image_);
                has_image_ = false;
            } else if ((now().seconds() - last_image_time_s_) > kImageTimeoutS) {
                display = cv::Mat(last_image_size_, CV_8UC3, cv::Scalar(0, 0, 0));
                signal_lost = true;
            } else {
                return;
            }
        }

        if (signal_lost) {
            cv::putText(display, "CAMERA SIGNAL LOST",
                        cv::Point(display.cols / 2 - 160, display.rows / 2),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        } else {
            drawSkeletons(display);
        }
        drawOverlay(display);

        cv::imshow("Azure Kinect FPV - Robot POV", display);
        cv::waitKey(1);
    }

    void drawSkeletons(cv::Mat& img) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (projected_people_.empty()) {
            return;
        }

        const double now_s = now().seconds();
        if ((now_s - last_camera_skeleton_time_s_) > kStaleDataTimeoutS ||
            (now_s - last_zone_time_s_) > kStaleDataTimeoutS) {
            return;
        }

        for (const auto& person : projected_people_) {
            if (active_body_ids_.find(person.first) == active_body_ids_.end()) {
                continue;
            }

            const cv::Scalar color = personColor(person.first);
            const auto& points = person.second;

            for (const auto& bone : kBones) {
                const auto a = points.find(bone.first);
                const auto b = points.find(bone.second);
                if (a == points.end() || b == points.end()) {
                    continue;
                }
                if (a->second.x < 0 || b->second.x < 0) {
                    continue;
                }
                cv::line(img, a->second, b->second, color, 3);
            }

            for (const auto& point : points) {
                const auto joint_color = joint_colors_.find(point.first);
                if (joint_color == joint_colors_.end()) {
                    continue;
                }
                if (point.second.x < 0) {
                    continue;
                }
                cv::circle(img, point.second, 6, joint_color->second, -1);
                cv::circle(img, point.second, 6, color, 1);
            }

            const auto head = points.find(ID_HEAD);
            const auto chest = points.find(ID_SPINE_CHEST);
            cv::Point anchor(-1, -1);
            if (head != points.end() && head->second.x >= 0) {
                anchor = head->second;
            } else if (chest != points.end() && chest->second.x >= 0) {
                anchor = chest->second;
            }
            if (anchor.x >= 0) {
                char label[16];
                snprintf(label, sizeof(label), "P%d", person.first);
                cv::putText(img, label, anchor + cv::Point(12, -12),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }
        }
    }

    void drawOverlay(cv::Mat& img) {
        const int font = cv::FONT_HERSHEY_SIMPLEX;
        const int x = 10;
        int y = 30;

        std::lock_guard<std::mutex> lock(mutex_);

        const cv::Scalar fps_color = (fps_ >= 10.0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
        cv::putText(img, "FPS: " + std::to_string(static_cast<int>(fps_)),
                    cv::Point(x, y), font, 0.6, fps_color, 2);
        y += 35;

        cv::Scalar status_color(0, 255, 0);
        if (safety_status_ == "WARNING") {
            status_color = cv::Scalar(0, 255, 255);
        } else if (safety_status_ == "CRITICAL") {
            status_color = cv::Scalar(0, 0, 255);
        }
        cv::putText(img, "Status: " + safety_status_, cv::Point(x, y), font, 0.6, status_color, 2);
        y += 35;

        cv::putText(img, "People in zone: " + std::to_string(active_body_ids_.size()),
                    cv::Point(x, y), font, 0.5, cv::Scalar(255, 255, 255), 1);
        y += 25;

        const bool base_fresh = (now().seconds() - last_base_time_s_) < kBaseDataTimeoutS;
        if (!base_fresh) {
            cv::putText(img, "Base data unavailable (check TF)",
                        cv::Point(x, y), font, 0.5, cv::Scalar(0, 0, 255), 1);
            return;
        }

        // Only show info for people currently in the workspace zone
        for (const auto& person : base_people_) {
            if (active_body_ids_.find(person.first) == active_body_ids_.end()) {
                continue;
            }

            const cv::Scalar color = personColor(person.first);
            for (const auto& entry : person.second) {
                if (entry.first != ID_HAND_LEFT && entry.first != ID_HAND_RIGHT &&
                    entry.first != ID_HEAD) {
                    continue;
                }
                const auto& p = entry.second;
                const double distance = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                char text[128];
                snprintf(text, sizeof(text), "P%d %s x:%.2f y:%.2f z:%.2f d:%.2fm",
                         person.first, jointName(entry.first), p.x, p.y, p.z, distance);
                cv::putText(img, text, cv::Point(x, y), font, 0.45, color, 1);
                y += 20;
            }
        }
    }

    std::unique_ptr<vision::frame::FrameTransformer> tf_transformer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr camera_skeleton_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr base_skeleton_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr zone_skeleton_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;
    cv::Mat current_image_;
    bool has_image_ = false;
    cv::Size last_image_size_ = cv::Size(640, 480);
    sensor_msgs::msg::CameraInfo camera_info_;
    bool has_camera_info_ = false;
    std::map<int, std::map<int, geometry_msgs::msg::Point>> camera_people_;
    std::map<int, std::map<int, cv::Point>> projected_people_;
    std::map<int, std::map<int, geometry_msgs::msg::Point>> base_people_;
    std::set<int> active_body_ids_;
    std::string camera_frame_id_;
    double last_base_time_s_ = -1.0;
    double last_zone_time_s_ = -1.0;
    double last_camera_skeleton_time_s_ = -1.0;
    double last_raw_msg_time_s_ = -1.0;
    bool using_compressed_ = false;
    std::string safety_status_ = "SAFE";
    double fps_ = 0.0;
    double last_image_time_s_ = 0.0;
    std::map<int, cv::Scalar> joint_colors_;
    double display_scale_ = 1.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualNode>());
    rclcpp::shutdown();
    return 0;
}