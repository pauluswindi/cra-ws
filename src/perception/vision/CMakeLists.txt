cmake_minimum_required(VERSION 3.8...3.22)
project(vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# Libraries
add_library(body_tracker_lib src/body_tracking/body_tracker.cpp)
ament_target_dependencies(body_tracker_lib
  rclcpp builtin_interfaces geometry_msgs visualization_msgs)

add_library(frame_transformer_lib src/frame/frame_transformer.cpp)
ament_target_dependencies(frame_transformer_lib
  rclcpp geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(frame_transformer_lib body_tracker_lib)

add_library(workspace_robot_lib src/workspace_robot/workspace_robot.cpp)
ament_target_dependencies(workspace_robot_lib rclcpp geometry_msgs)
target_link_libraries(workspace_robot_lib body_tracker_lib)

# Executables
add_executable(skeleton_transform_node src/nodes/skeleton_transform_node.cpp)
ament_target_dependencies(skeleton_transform_node
  rclcpp geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(skeleton_transform_node body_tracker_lib frame_transformer_lib)

add_executable(workspace_monitor_node src/nodes/workspace_monitor_node.cpp)
ament_target_dependencies(workspace_monitor_node
  rclcpp geometry_msgs std_msgs visualization_msgs)
target_link_libraries(workspace_monitor_node body_tracker_lib workspace_robot_lib)

add_executable(visual_node src/nodes/visual_node.cpp)
ament_target_dependencies(visual_node
  rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs cv_bridge
  tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(visual_node frame_transformer_lib ${OpenCV_LIBS})

# Install libraries
install(TARGETS
  body_tracker_lib
  frame_transformer_lib
  workspace_robot_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install executables
install(TARGETS
  skeleton_transform_node
  workspace_monitor_node
  visual_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(body_tracker_lib frame_transformer_lib workspace_robot_lib)
ament_export_dependencies(
  rclcpp builtin_interfaces sensor_msgs std_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs cv_bridge)

ament_package()