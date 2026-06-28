cmake_minimum_required(VERSION 3.8)
project(libi_fleet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(libi_fleet_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- core 라이브러리 (navgraph) ---
add_library(libi_fleet_core src/navgraph.cpp)
target_include_directories(libi_fleet_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(libi_fleet_core yaml-cpp)
ament_target_dependencies(libi_fleet_core rclcpp)

# --- pluginlib 플러그인 (dispatcher/traffic) ---
add_library(libi_fleet_plugins SHARED
  plugins/greedy_cost.cpp
  plugins/farthest_cost.cpp
  plugins/edge_node_lock.cpp
  plugins/no_lock.cpp)
target_include_directories(libi_fleet_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(libi_fleet_plugins libi_fleet_core)
ament_target_dependencies(libi_fleet_plugins pluginlib rclcpp)
pluginlib_export_plugin_description_file(libi_fleet plugins.xml)

# --- fleet 노드 ---
add_executable(fleet_node src/fleet_node.cpp)
target_include_directories(fleet_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(fleet_node libi_fleet_core)
ament_target_dependencies(fleet_node
  rclcpp pluginlib libi_fleet_msgs rmf_fleet_msgs geometry_msgs)

install(TARGETS libi_fleet_core libi_fleet_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS fleet_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
