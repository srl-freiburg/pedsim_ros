CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(pedsim_simulator)

set( PEDSIM_CORE_DEPENDENCIES
    roscpp
    rospy
    std_msgs
    pedsim_msgs
    pedsim_srvs
    visualization_msgs
    nav_msgs
    geometry_msgs
    tf
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${PEDSIM_CORE_DEPENDENCIES})

# add qt4 to the project
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

INCLUDE(${QT_USE_FILE})

catkin_package(
 CATKIN_DEPENDS ${PEDSIM_CORE_DEPENDENCIES}
)

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libpedsim)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libpedsim)


include_directories(include)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
)

set(SOURCES 
    src/scene.cpp 
    src/agent.cpp 
    src/obstacle.cpp 
    src/waypoint.cpp 
    src/config.cpp 
    src/waitingqueue.cpp 
    src/orientationhandler.cpp
) 

add_executable(simulator ${SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_executable(teleop src/robot_teleop.cpp)

TARGET_LINK_LIBRARIES(simulator 
    ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${QWT_LIBRARIES} pedsim 
    ${BOOST_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES}
)

TARGET_LINK_LIBRARIES(teleop ${BOOST_LIBRARIES} ${catkin_LIBRARIES})


INSTALL(TARGETS simulator teleop
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)
