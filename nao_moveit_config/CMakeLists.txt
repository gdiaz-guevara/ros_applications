cmake_minimum_required(VERSION 3.8)
project(nao_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/nao_moveit_py.py DESTINATION lib/${PROJECT_NAME})

ament_package()
