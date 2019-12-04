cmake_minimum_required(VERSION 2.8.3)
project(pedsim_sensors)
add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)

set(PACKAGE_DEPS
  geometry_msgs
  nav_msgs
  pedsim_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  pedsim_utils
)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPS})

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

# People point cloud sensor.
set(PEOPLE_PCD_EXEC_NAME pedsim_people_sensor)
add_executable(${PEOPLE_PCD_EXEC_NAME} src/pedsim_sensors/people_point_cloud.cpp)
add_dependencies(${PEOPLE_PCD_EXEC_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PEOPLE_PCD_EXEC_NAME} ${catkin_LIBRARIES})

# Obstacle point cloud sensor.
set(OBSTACLE_PCD_EXEC_NAME pedsim_obstacle_sensor)
add_executable(${OBSTACLE_PCD_EXEC_NAME} src/pedsim_sensors/obstacle_point_cloud.cpp)
add_dependencies(${OBSTACLE_PCD_EXEC_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${OBSTACLE_PCD_EXEC_NAME} ${catkin_LIBRARIES})

# Point cloud sensor.
set(OCCLUSION_PCD_EXEC_NAME pedsim_occlusion_sensor)
add_executable(${OCCLUSION_PCD_EXEC_NAME} src/pedsim_sensors/occlusion_point_cloud.cpp)
add_dependencies(${OCCLUSION_PCD_EXEC_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${OCCLUSION_PCD_EXEC_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS
    ${PEOPLE_PCD_EXEC_NAME}
    ${OBSTACLE_PCD_EXEC_NAME}
    ${OCCLUSION_PCD_EXEC_NAME}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pedsim_sensors.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
