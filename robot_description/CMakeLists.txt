cmake_minimum_required(VERSION 3.5)

project(robot_description)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python scripts
install(PROGRAMS
  scripts/scan_clipper.py
  scripts/imu_covariance_injector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    models
    worlds
    rviz
    config
    urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()