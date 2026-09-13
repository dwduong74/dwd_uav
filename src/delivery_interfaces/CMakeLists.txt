cmake_minimum_required(VERSION 3.8)
project(delivery_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerDetection.msg" "msg/MissionStatus.msg" "msg/LandingTarget.msg"
  "msg/FlightIntent.msg" "msg/FlightState.msg" "msg/PayloadState.msg"
  "action/ExecuteDelivery.action" "action/ReleasePayload.action"
  DEPENDENCIES std_msgs geometry_msgs unique_identifier_msgs builtin_interfaces action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
