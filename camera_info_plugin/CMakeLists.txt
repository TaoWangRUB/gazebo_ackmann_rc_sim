cmake_minimum_required(VERSION 3.10)
project(camera_info_plugin)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

# Find ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find Ignition dependencies
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-common4 REQUIRED)

# Create the plugin library
add_library(${PROJECT_NAME} SHARED 
  src/camera_info_plugin.cpp
)

# Set target dependencies using ament
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  ignition-gazebo6
  ignition-plugin1
  ignition-common4
)

# Install to the workspace lib directory (not package-specific)
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()