cmake_minimum_required(VERSION 2.8.3)
project(pedsim_visualizer)
add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)

set(PACKAGE_DEPS
  geometry_msgs
  pedsim_msgs
  pedsim_utils
  roscpp
  rospy
  std_msgs
  visualization_msgs
  dynamic_reconfigure
)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPS})

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  config/PedsimVisualizer.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPS}
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(EXECUTABLE_NAME ${PROJECT_NAME}_node)
add_executable(${EXECUTABLE_NAME}
  src/sim_visualizer_node.cpp
  src/sim_visualizer.cpp
)

add_dependencies(${EXECUTABLE_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(${EXECUTABLE_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${EXECUTABLE_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${EXECUTABLE_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pedsim_visualizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
