cmake_minimum_required(VERSION 2.8.3)
project(pedsim_gazebo_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  gazebo_msgs
  pedsim_msgs
  pedsim_utils
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
 find_package(gazebo REQUIRED )
 link_directories(${GAZEBO_LIBRARY_DIRS})


################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gzb_vel_plugin
  CATKIN_DEPENDS gazebo_ros roscpp  geometry_msgs pedsim_msgs pedsim_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


add_library(ActorPosesPlugin src/actor_poses_plugin.cpp)
target_link_libraries(ActorPosesPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


#############
## Install ##
#############

