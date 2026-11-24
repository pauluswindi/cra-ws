cmake_minimum_required(VERSION 3.8...3.22)
project(vision)

# Real-time image processing should not default to an unoptimized build.
# Keep debug symbols and honor an explicitly selected build configuration.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build configuration" FORCE)
endif()

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
find_package(msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# Body Tracking
add_library(body_tracker_lib
  src/body_tracking/body_tracker.cpp
)
ament_target_dependencies(body_tracker_lib
  rclcpp builtin_interfaces geometry_msgs visualization_msgs
)

# Frame Transformation
add_library(frame_transformer_lib
  src/frame/frame_transformer.cpp
)
ament_target_dependencies(frame_transformer_lib
  rclcpp geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(frame_transformer_lib
  body_tracker_lib
)

# Workspace Robot
add_library(workspace_robot_lib
  src/workspace_robot/workspace_robot.cpp
)
ament_target_dependencies(workspace_robot_lib
  rclcpp geometry_msgs
)
target_link_libraries(workspace_robot_lib
  body_tracker_lib
)

# ROI Generation
add_library(roi_generator_lib
  src/reconstruction/roi_generator.cpp
)
ament_target_dependencies(roi_generator_lib
  geometry_msgs
)

# Depth Processing
add_library(depth_processor_lib
  src/reconstruction/depth_processor.cpp
)
ament_target_dependencies(depth_processor_lib
  sensor_msgs geometry_msgs tf2
)
target_link_libraries(depth_processor_lib
  ${OpenCV_LIBS}
)

# Voxel Reconstruction
add_library(voxel_reconstruction_lib
  src/reconstruction/voxel_reconstruction.cpp
)
ament_target_dependencies(voxel_reconstruction_lib
  geometry_msgs
)

# Human Segmentation
add_library(human_segmenter_lib
  src/segmentation/human_segmenter.cpp
)
ament_target_dependencies(human_segmenter_lib
  sensor_msgs
)

# Human Capsule
add_library(human_capsule_lib
  src/human/human_capsule.cpp
)
ament_target_dependencies(human_capsule_lib
  geometry_msgs
)

# Reconstruction Pipeline
add_library(reconstruction_pipeline_lib
  src/reconstruction/reconstruction_pipeline.cpp
)
ament_target_dependencies(reconstruction_pipeline_lib
  sensor_msgs geometry_msgs
)
target_link_libraries(reconstruction_pipeline_lib
  roi_generator_lib
  depth_processor_lib
  voxel_reconstruction_lib
  human_segmenter_lib
  human_capsule_lib
)

# Skeleton Transform Node
add_executable(skeleton_transform_node
  src/nodes/skeleton_transform_node.cpp
)
ament_target_dependencies(skeleton_transform_node
  rclcpp geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(skeleton_transform_node
  body_tracker_lib
  frame_transformer_lib
)

# Workspace Monitor Node
add_executable(workspace_monitor_node
  src/nodes/workspace_monitor_node.cpp
)
ament_target_dependencies(workspace_monitor_node
  rclcpp geometry_msgs std_msgs visualization_msgs
)
target_link_libraries(workspace_monitor_node
  body_tracker_lib
  workspace_robot_lib
)

# Visualization Node
add_executable(visual_node
  src/nodes/visual_node.cpp
)
ament_target_dependencies(visual_node
  rclcpp sensor_msgs std_msgs geometry_msgs
  visualization_msgs cv_bridge
  tf2 tf2_ros tf2_geometry_msgs
)
target_link_libraries(visual_node
  frame_transformer_lib
  ${OpenCV_LIBS}
)

# Reconstruction Node
add_executable(reconstruction_node
  src/nodes/reconstruction_node.cpp
)
ament_target_dependencies(reconstruction_node
  rclcpp
  builtin_interfaces
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  msgs
)
target_link_libraries(reconstruction_node
  reconstruction_pipeline_lib
)

# Install Libraries
install(TARGETS
  body_tracker_lib
  frame_transformer_lib
  workspace_robot_lib
  roi_generator_lib
  depth_processor_lib
  voxel_reconstruction_lib
  human_segmenter_lib
  human_capsule_lib
  reconstruction_pipeline_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install Executables
install(TARGETS
  skeleton_transform_node
  workspace_monitor_node
  visual_node
  reconstruction_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Headers
install(DIRECTORY include/
  DESTINATION include
)

# Install Launch Files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(vision_stream_probe test/vision_stream_probe.cpp)
  find_package(trajectory_msgs REQUIRED)
  ament_target_dependencies(vision_stream_probe rclcpp sensor_msgs visualization_msgs msgs
    geometry_msgs std_msgs trajectory_msgs)
  ament_add_gtest(test_preview_decode test/test_preview_decode.cpp)
  target_link_libraries(test_preview_decode ${OpenCV_LIBS})
  add_executable(benchmark_preview_decode test/benchmark_preview_decode.cpp)
  target_link_libraries(benchmark_preview_decode ${OpenCV_LIBS})
  ament_add_gtest(test_sparse_depth test/test_sparse_depth.cpp)
  ament_target_dependencies(test_sparse_depth sensor_msgs geometry_msgs)
  target_link_libraries(test_sparse_depth
    depth_processor_lib human_segmenter_lib reconstruction_pipeline_lib)

  ament_add_gtest(test_frame_batch test/test_frame_batch.cpp)
  ament_target_dependencies(test_frame_batch rclcpp geometry_msgs tf2 tf2_ros)
  target_link_libraries(test_frame_batch frame_transformer_lib)
  set_tests_properties(test_frame_batch PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=230;ROS_LOCALHOST_ONLY=1")

  add_executable(benchmark_sparse_depth test/benchmark_sparse_depth.cpp)
  ament_target_dependencies(benchmark_sparse_depth sensor_msgs geometry_msgs)
  target_link_libraries(benchmark_sparse_depth
    depth_processor_lib human_segmenter_lib reconstruction_pipeline_lib)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_libraries(
  body_tracker_lib
  frame_transformer_lib
  workspace_robot_lib
  roi_generator_lib
  depth_processor_lib
  voxel_reconstruction_lib
  human_segmenter_lib
  human_capsule_lib
  reconstruction_pipeline_lib
)

ament_export_dependencies(
  rclcpp
  builtin_interfaces
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

ament_package()
