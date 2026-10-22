cmake_minimum_required(VERSION 3.5)
project(cra)

# Temukan dependency
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Instalasi model robot
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Instalasi file konfigurasi
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Instalasi file launch
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
