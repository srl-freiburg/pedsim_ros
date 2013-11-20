CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(simulator)

# Hide CMake variables
SET (CMAKE_BUILD_TYPE "" CACHE INTERNAL "" FORCE)

#------------------ Configuration ------------------#
#Debugging (enable: ON, disable: OFF)
OPTION(SHALL_DEBUG "Enable debug features" OFF)

#Profiling (enable: ON, disable: OFF)
OPTION(SHALL_PROFILE "Enable the code profiling feature" OFF)

#Full compiler output (enable: ON, disable: OFF)
OPTION(CMAKE_VERBOSE_MAKEFILE "Full compiler output" ON)
#---------------- End Configuration ----------------#

# Set output directories for libraries and executables
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)


#----------------- Compiler Flags ------------------#
ADD_DEFINITIONS(-std=c++0x -pipe)  # suppress the noise of unused variables

#debugging
IF(SHALL_DEBUG)
	MESSAGE("Debugging activated")
        ADD_DEFINITIONS(-O0 -DDEBUG -ggdb -g3 -rdynamic)
ELSE(SHALL_DEBUG)
	MESSAGE("Debugging deactivated")
	ADD_DEFINITIONS(-Os)
ENDIF(SHALL_DEBUG)

#--------------- End Compiler Flags ----------------#



# add qt4 to the project
# enable additional qt modules:
SET(QT_USE_QTOPENGL TRUE)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})




# add PedSim to the application
SET(PedSim_DIR ${CMAKE_CURRENT_SOURCE_DIR})
FIND_PACKAGE(PedSim)
IF(NOT PEDSIM_FOUND)
	MESSAGE(SEND_ERROR "Couldn't find PedSim!")
	MESSAGE("Please edit PedSimConfig.cmake according to your installation.")
	MESSAGE("You need to set the variables: PEDSIM_INCLUDE_DIR and PEDSIM_LIBRARY.")
ENDIF(NOT PEDSIM_FOUND)
INCLUDE_DIRECTORIES(${PEDSIM_INCLUDE_DIR})


INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/3rdparty/)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})


# search library
find_package(Boost REQUIRED)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)

SET(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR})     # for using additional FindX.cmake files
FIND_PACKAGE(Qwt REQUIRED)
find_package(OpenCV REQUIRED)



file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

### CATKIN
set( PEDSIM_CORE_DEPENDENCIES
    roscpp
    rospy
    pedsim_msgs
    pedsim_srvs
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ${PEDSIM_CORE_DEPENDENCIES}
)


catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${PEDSIM_CORE_DEPENDENCIES}
)

add_executable(simulator ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

TARGET_LINK_LIBRARIES(simulator
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    ${QWT_LIBRARIES}
    ${PEDSIM_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)


IF(SHALL_PROFILE)
    MESSAGE("Profiling activated")
    SET_TARGET_PROPERTIES(simulator PROPERTIES COMPILE_FLAGS -pg LINK_FLAGS -pg)
ENDIF(SHALL_PROFILE)

#------------------- Installation ------------------#

INSTALL(TARGETS simulator
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

# Resources (scenarios etc.)
INSTALL(DIRECTORY resources/ DESTINATION resources)
#---------------- End Installation -----------------#
