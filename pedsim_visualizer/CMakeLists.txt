cmake_minimum_required(VERSION 3.5)
project(pedsim_visualizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

set(dependencies
  rclcpp
  rclpy
  std_msgs
  pedsim_msgs
  pedsim_utils
  visualization_msgs
  spencer_tracking_msgs
  geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(pedsim_utils REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(spencer_tracking_msgs REQUIRED)
find_package(Boost REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

#generate_dynamic_reconfigure_options(
#  config/PedsimVisualizer.cfg
#)

include_directories(include)

set(EXECUTABLE_NAME ${PROJECT_NAME}_node)
add_executable(${EXECUTABLE_NAME}
  src/sim_visualizer_node.cpp
  src/sim_visualizer.cpp
)
ament_target_dependencies(${EXECUTABLE_NAME} ${dependencies})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${EXECUTABLE_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()

