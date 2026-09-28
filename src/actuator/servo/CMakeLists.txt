cmake_minimum_required(VERSION 3.8)
project(servo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(msgs REQUIRED)

add_library(servo_component SHARED
  src/servo_component.cpp
)

target_compile_features(servo_component PUBLIC cxx_std_20)

target_include_directories(servo_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(servo_component
  dynamixel_sdk
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  msgs
)

target_link_libraries(servo_component
  dynamixel_sdk::dynamixel_sdk
)

rclcpp_components_register_nodes(servo_component
  "servo_driver::ServoComponent"
)

install(TARGETS servo_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  dynamixel_sdk
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  msgs
)
ament_export_include_directories(include)

ament_package()