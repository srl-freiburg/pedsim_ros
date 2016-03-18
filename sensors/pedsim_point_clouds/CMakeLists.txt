cmake_minimum_required(VERSION 2.8.3)

project(pedsim_point_clouds)

ADD_DEFINITIONS(-Wall -Wunused -std=c++0x -pipe) # C++ 11 is required

set( PACKAGE_DEPENDENCIES
    roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    spencer_tracking_msgs
    tf
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} )


add_executable(pedsim_point_clouds src/pedsim_point_clouds.cpp)
add_dependencies(pedsim_point_clouds ${catkin_EXPORTED_TARGETS})
target_link_libraries(pedsim_point_clouds ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS pedsim_point_clouds
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



