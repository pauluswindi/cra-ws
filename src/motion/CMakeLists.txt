cmake_minimum_required(VERSION 3.8)
project(motion)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(COMMON_HEADERS_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/../common
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# MuJoCo installed through Python
set(MUJOCO_ROOT
  "/usr/local/lib/python3.10/dist-packages/mujoco"
  CACHE PATH "MuJoCo installation directory"
)

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  PATHS "${MUJOCO_ROOT}/include"
  NO_DEFAULT_PATH
  REQUIRED
)

find_library(MUJOCO_LIBRARY
  NAMES libmujoco.so.3.1.6
  PATHS "${MUJOCO_ROOT}"
  NO_DEFAULT_PATH
  REQUIRED
)

add_library(mujoco::mujoco SHARED IMPORTED)

set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)

message(STATUS "MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo library: ${MUJOCO_LIBRARY}")

# Kinematics and trajectory library
add_library(robot_kinematics STATIC
  src/kinematics/robot_kinematics.cpp
  src/kinematics/jacobian.cpp
  src/trajectory/motion_profile.cpp
  src/trajectory/trajectory_generator.cpp
)

target_compile_features(robot_kinematics PUBLIC cxx_std_20)

target_include_directories(robot_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(robot_kinematics PUBLIC
  Eigen3::Eigen
  mujoco::mujoco
)

set_target_properties(robot_kinematics PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# Motion component
add_library(motion_component SHARED
  src/motion_component/motion_component.cpp

  src/controllers/cartesian_controller.cpp
  src/controllers/joint_controller.cpp
  src/controllers/execution_monitor.cpp

  src/modules/data_module.cpp
  src/modules/joint_feedback.cpp
  src/modules/motion_guard.cpp
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
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  visualization_msgs
  msgs
  ament_index_cpp
)

target_link_libraries(motion_component
  robot_kinematics
  yaml-cpp
)

rclcpp_components_register_nodes(motion_component
  "motion::MotionComponent"
)

# Kinematics test
add_executable(test_robot_kinematics
  test/test_kinematics.cpp
)

target_link_libraries(test_robot_kinematics PRIVATE
  robot_kinematics
)

# Jacobian and IK test
add_executable(test_jacobian
  test/test_jacobian.cpp
)

target_link_libraries(test_jacobian PRIVATE
  robot_kinematics
)

# Install libraries
install(
  TARGETS
    motion_component
    robot_kinematics
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install test executables
install(
  TARGETS
    test_robot_kinematics
    test_jacobian
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install common headers
install(
  DIRECTORY ${COMMON_HEADERS_PATH}/
  DESTINATION include/${PROJECT_NAME}/common
  FILES_MATCHING PATTERN "*.hpp"
)

# Install motion headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  msgs
  Eigen3
)

ament_export_include_directories(include)

ament_package()