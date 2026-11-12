cmake_minimum_required(VERSION 3.8)
project(nao_gz_controller)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf worlds meshes materials
  DESTINATION share/${PROJECT_NAME}/
)

# Example nodes (Python) installed as executables: ros2 run nao_gz_controller <name>
install(
  PROGRAMS scripts/nao_wave.py scripts/head_sweep.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
