cmake_minimum_required(VERSION 3.8)
project(libi_fleet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libi_fleet_msgs REQUIRED)

add_executable(fleet_node src/fleet_node.cpp)
ament_target_dependencies(fleet_node rclcpp libi_fleet_msgs)

install(TARGETS fleet_node DESTINATION lib/${PROJECT_NAME})

ament_package()
