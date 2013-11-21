CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(PedSim)

#------------------ Configuration ------------------#
#Debugging (enable: ON, disable: OFF)
OPTION(SHALL_DEBUG "Enable debug features" ON)

#Profiling (enable: ON, disable: OFF)
OPTION(SHALL_PROFILE "Enable the code profiling feature" OFF)

#Full compiler output (enable: ON, disable: OFF)
OPTION(CMAKE_VERBOSE_MAKEFILE "Full compiler output" ON)
#---------------- End Configuration ----------------#


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

#--------------- End Compiler Flags ----------------#


#----------------------- Files ---------------------#
#source code files
SET(SRC_FILES
    ped_vector.cpp
    ped_obstacle.cpp
    ped_waypoint.cpp
    ped_agent.cpp
    ped_scene.cpp
    ped_tree.cpp
)
#--------------------- End Files -------------------#

# We need to add the current the source directory itself to the include path
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})


#create executables
ADD_LIBRARY(pedsim SHARED ${SRC_FILES})

IF(SHALL_PROFILE)
    MESSAGE("Profiling activated")
    SET_TARGET_PROPERTIES(pedsim PROPERTIES COMPILE_FLAGS -pg LINK_FLAGS -pg)
ENDIF(SHALL_PROFILE)

#------------------- Installation ------------------#
INSTALL(TARGETS pedsim
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    DESTINATION include)
#---------------- End Installation -----------------#