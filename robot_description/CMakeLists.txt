cmake_minimum_required(VERSION 3.5)

project(robot_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

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