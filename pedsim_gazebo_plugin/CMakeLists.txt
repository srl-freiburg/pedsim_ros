cmake_minimum_required(VERSION 3.5)
project(pedsim_gazebo_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include
  ${geometry_msgs_INCLUDE_DIRS}
  ${pedsim_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

set(dependencies
    rclcpp
    pedsim_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2
    gazebo_msgs
    ament_index_cpp
)

add_executable(spawn_pedsim_agents src/spawn_pedsim_agents.cpp)
ament_target_dependencies(spawn_pedsim_agents ${dependencies})

install(TARGETS
spawn_pedsim_agents
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ActorPosesPlugin
add_library(gazebo_ros_actor_poses SHARED
  src/actor_poses_plugin.cpp
)

ament_target_dependencies(gazebo_ros_actor_poses
  "gazebo_ros"
  "rclcpp"
  "pedsim_msgs"
  "tf2"
  "tf2_geometry_msgs"
)
ament_export_libraries(gazebo_ros_actor_poses)

ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(gazebo_plugins)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(pedsim_msgs)

install(
  PROGRAMS
    scripts/pedsim_to_gazebo_world.py
    scripts/spawn_pedsim_agents.py
    scripts/spawn_single_agent.py
    DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
  gazebo_ros_actor_poses
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds models
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()