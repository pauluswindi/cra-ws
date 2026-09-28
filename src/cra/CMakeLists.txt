cmake_minimum_required(VERSION 3.5)
project(cra)

# Temukan dependency
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Instalasi file launch
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
