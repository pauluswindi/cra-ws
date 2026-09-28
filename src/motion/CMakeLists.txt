cmake_minimum_required(VERSION 3.8)
project(motion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Compiler warning flags for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(COMMON_HEADERS_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(msgs REQUIRED)

add_library(motion_component SHARED
  src/motion_component/motion_component.cpp
  src/modules/data_module.cpp
  src/modules/servo_module.cpp
  src/logic/robot_initializer.cpp
)

target_compile_features(motion_component PUBLIC cxx_std_20)

target_include_directories(motion_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${COMMON_HEADERS_PATH}>
)

ament_target_dependencies(motion_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  msgs
)

rclcpp_components_register_nodes(motion_component
  "motion::MotionComponent"
)

install(TARGETS motion_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY ${COMMON_HEADERS_PATH}/
  DESTINATION include/${PROJECT_NAME}/common
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  msgs
)
ament_export_include_directories(include)

ament_package()