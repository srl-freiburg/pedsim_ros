cmake_minimum_required(VERSION 2.8.3)
project(pedsim)




#------------------ Configuration ------------------#
#Debugging (enable: ON, disable: OFF)
OPTION(SHALL_DEBUG "Enable debug features" OFF)

#Profiling (enable: ON, disable: OFF)
OPTION(SHALL_PROFILE "Enable the code profiling feature" OFF)

#Full compiler output (enable: ON, disable: OFF)
OPTION(CMAKE_VERBOSE_MAKEFILE "Full compiler output" ON)


#----------------- Compiler Flags ------------------#
#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall -Wunused -std=c++0x -pipe)

#debugging
IF(SHALL_DEBUG)
    MESSAGE("Debugging activated")
    ADD_DEFINITIONS(-O0 -DDEBUG -ggdb -g3 -rdynamic)
ELSE(SHALL_DEBUG)
    MESSAGE("Debugging deactivated")
    ADD_DEFINITIONS(-Os)
ENDIF(SHALL_DEBUG)


FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
)


FIND_PACKAGE(Boost REQUIRED)


catkin_package(
  CATKIN_DEPENDS  roscpp 
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/
  LIBRARIES pedsim
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/pedsim/
  ${roscpp_INCLUDE_DIRS}
)


set(SOURCES 
     src/ped_agent.cpp src/ped_angle.cpp src/ped_obstacle.cpp src/ped_scene.cpp src/ped_tree.cpp src/ped_vector.cpp src/ped_waypoint.cpp
) 



add_library(pedsim ${SOURCES})

TARGET_LINK_LIBRARIES(pedsim 
    ${BOOST_LIBRARIES}  
)
