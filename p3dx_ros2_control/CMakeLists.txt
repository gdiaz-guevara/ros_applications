cmake_minimum_required(VERSION 3.8)
project(p3dx_ros2_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS scripts/square_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
