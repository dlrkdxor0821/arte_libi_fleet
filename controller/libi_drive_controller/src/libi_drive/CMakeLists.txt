cmake_minimum_required(VERSION 3.8)
project(libi_drive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(libi_fleet_msgs REQUIRED)

add_executable(drive_node src/drive_node.cpp)
ament_target_dependencies(drive_node rclcpp rclcpp_action libi_fleet_msgs)

install(TARGETS drive_node DESTINATION lib/${PROJECT_NAME})

ament_package()
