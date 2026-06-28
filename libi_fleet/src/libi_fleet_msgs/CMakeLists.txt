cmake_minimum_required(VERSION 3.8)
project(libi_fleet_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskState.msg"
  "msg/RobotState.msg"
  "srv/SubmitTask.srv"
  "action/Navigate.action"
  "action/PerformAction.action"
  DEPENDENCIES geometry_msgs
)

ament_package()
