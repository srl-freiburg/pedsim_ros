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
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${pedsim_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

link_directories(${gazebo_dev_LIBRARY_DIRS})


ament_python_install_package(scripts)

# ActorPosesPlugin
add_library(gazebo_ros_actor_poses SHARED
  src/actor_poses_plugin.cpp
)

ament_target_dependencies(gazebo_ros_actor_poses
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "pedsim_msgs"
  "tf2"
  "tf2_geometry_msgs"
)
ament_export_libraries(gazebo_ros_actor_poses)

ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(pedsim_msgs)

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

ament_package()


install(
  PROGRAMS
    scripts/pedsim_to_gazebo_world.py
    scripts/spawn_pedsim_agents.py
    scripts/spawn_single_agent.py
    DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS
  gazebo_ros_actor_poses
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

