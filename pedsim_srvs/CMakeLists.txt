cmake_minimum_required(VERSION 3.5)
project(pedsim_srvs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetAgentState.srv"
  "srv/GetAllAgentsState.srv"
  "srv/SetAgentState.srv"
  "srv/SetAllAgentsState.srv"
  DEPENDENCIES builtin_interfaces std_msgs pedsim_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()