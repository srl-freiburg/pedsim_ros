cmake_minimum_required(VERSION 3.5)
project(pedsim_tf2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
set(dependencies
    rclcpp
    pedsim_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2
)

include_directories(include)

add_executable(pedsim_tf2_node src/pedsim_tf2_node.cpp)
ament_target_dependencies(pedsim_tf2_node ${dependencies})

install(TARGETS
  pedsim_tf2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
