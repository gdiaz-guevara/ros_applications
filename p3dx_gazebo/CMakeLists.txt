cmake_minimum_required(VERSION 3.8)
project(p3dx_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

install(
  DIRECTORY
    config
    launch
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
