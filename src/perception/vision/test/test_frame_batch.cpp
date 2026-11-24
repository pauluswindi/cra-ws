#include <chrono>
#include <cmath>
#include <thread>

#include "gtest/gtest.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "vision/frame/frame_transformer.hpp"

TEST(FrameBatch, OneSnapshotMatchesPerJointLookupAndMissingTfIsRejected)
{
    rclcpp::init(0, nullptr);
    {
        auto node = std::make_shared<rclcpp::Node>("vision_transform_batch_test");
        vision::frame::FrameTransformer transformer(node.get());
        tf2_ros::StaticTransformBroadcaster broadcaster(node);
        geometry_msgs::msg::TransformStamped message;
        message.header.frame_id = "test_rgb";
        message.child_frame_id = "test_depth";
        message.header.stamp = node->now();
        message.transform.translation.x = .03;
        message.transform.translation.y = -.02;
        message.transform.translation.z = .01;
        message.transform.rotation.y = std::sin(.15);
        message.transform.rotation.w = std::cos(.15);
        broadcaster.sendTransform(message);
        tf2::Transform batch;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        bool ready = false;
        while (std::chrono::steady_clock::now() < deadline) {
            ready = transformer.lookupTransform("test_rgb", "test_depth", batch);
            if (ready) { break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        EXPECT_TRUE(ready);
        if (ready) {
            for (int i = 0; i < 96; ++i) {
                geometry_msgs::msg::Point point;
                point.x = i * .002;
                point.y = i * -.003;
                point.z = 1. + i * .001;
                const auto old = transformer.transformPoint(point, "test_rgb", "test_depth");
                const auto current = batch * tf2::Vector3(point.x, point.y, point.z);
                EXPECT_DOUBLE_EQ(old.x, current.x());
                EXPECT_DOUBLE_EQ(old.y, current.y());
                EXPECT_DOUBLE_EQ(old.z, current.z());
            }
        }
        EXPECT_FALSE(transformer.lookupTransform("unknown_rgb", "test_depth", batch));
        EXPECT_TRUE(transformer.lookupTransform("test_depth", "test_depth", batch));
        const auto same = batch * tf2::Vector3(.1, .2, .3);
        EXPECT_DOUBLE_EQ(same.x(), .1);
        EXPECT_DOUBLE_EQ(same.y(), .2);
        EXPECT_DOUBLE_EQ(same.z(), .3);
    }
    rclcpp::shutdown();
}
