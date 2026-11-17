// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <sstream>

// Library headers
//
#include "rclcpp/rclcpp.hpp"
#include <k4a/k4a.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Create Node for handling info and error messages
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("k4a_bridge");


  // Setup the K4A device
  std::shared_ptr<K4AROSDevice> device;
  int exit_code = 0;
  try
  {
    device = std::make_shared<K4AROSDevice>();
    if (device->startCameras() != K4A_RESULT_SUCCEEDED)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to start cameras");
      exit_code = 1;
    }
    else if (device->startImu() != K4A_RESULT_SUCCEEDED)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to start IMU");
      exit_code = 2;
    }
    else
    {
      RCLCPP_INFO(node->get_logger(), "K4A Started");
      // Camera-node callbacks are still serviced by its existing frame worker.
      rclcpp::spin(node);
      RCLCPP_INFO(node->get_logger(), "ROS Exit Started");
    }
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "K4A initialization/runtime failure: %s", error.what());
    exit_code = 3;
  }

  device.reset();

  RCLCPP_INFO(node->get_logger(),"ROS Exit");

  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  RCLCPP_INFO(node->get_logger(),"ROS Shutdown complete");

  RCLCPP_INFO(node->get_logger(),"Finished ros bridge main");
  return exit_code;
}
