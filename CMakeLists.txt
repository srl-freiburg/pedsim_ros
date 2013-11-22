CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(simulator)

set( PEDSIM_CORE_DEPENDENCIES
    roscpp
    rospy
    std_msgs
    pedsim_msgs
    pedsim_srvs
    visualization_msgs
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${PEDSIM_CORE_DEPENDENCIES})

# add qt4 to the project
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

catkin_package(
 CATKIN_DEPENDS ${PEDSIM_CORE_DEPENDENCIES}
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libpedsim)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libpedsim
)

# file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)
set(SOURCES 
    src/scene.cpp src/cell.cpp src/agent.cpp src/obstacle.cpp src/waypoint.cpp src/config.cpp
) 

add_executable(simulator ${SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

TARGET_LINK_LIBRARIES(simulator 
    ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QWT_LIBRARIES} pedsim 
    ${BOOST_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES}
)


INSTALL(TARGETS simulator
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)
