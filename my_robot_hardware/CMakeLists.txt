cmake_minimum_required(VERSION 3.16)
project(my_robot_hardware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(my_robot_hardware SHARED src/diffbot_system.cpp)
target_include_directories(my_robot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(my_robot_hardware PUBLIC
  hardware_interface::hardware_interface pluginlib::pluginlib
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)

# Tell pluginlib that this library provides a hardware_interface plugin
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware.xml)

install(TARGETS my_robot_hardware LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_package()
