cmake_minimum_required(VERSION 2.8)
project(pedsim_simulator)
add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)
set(PEDSIM_SIMULATOR_DEPENDENCIES
  roscpp
  rospy
  std_msgs
  pedsim
  pedsim_msgs
  pedsim_srvs
  pedsim_utils
  std_srvs
  geometry_msgs
  nav_msgs
  tf
  cmake_modules
  dynamic_reconfigure
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${PEDSIM_SIMULATOR_DEPENDENCIES})
find_package(Boost REQUIRED)
find_package(Qt5Widgets REQUIRED)

# dynamic reconfigure parameters
generate_dynamic_reconfigure_options(config/PedsimSimulator.cfg)

catkin_package(
  CATKIN_DEPENDS ${PEDSIM_SIMULATOR_DEPENDENCIES}
  INCLUDE_DIRS include
)

include_directories(include)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDES})
add_definitions(${Qt5Widgets_DEFINITIONS})

set(SOURCES
  	src/simulator_node.cpp
	src/simulator.cpp
  	src/scene.cpp
  	src/config.cpp
  	src/agentstatemachine.cpp
  	src/scenarioreader.cpp
	src/rng.cpp

	# elements
	src/element/agent.cpp
	src/element/agentgroup.cpp
	src/element/agentcluster.cpp
	src/element/areawaypoint.cpp
	src/element/attractionarea.cpp
	src/element/queueingwaypoint.cpp
	src/element/waitingqueue.cpp
	src/element/waypoint.cpp
	src/element/obstacle.cpp
	src/element/scenarioelement.cpp

	# forces
	src/force/alongwallforce.cpp
	src/force/force.cpp
	src/force/groupcoherenceforce.cpp
	src/force/groupgazeforce.cpp
	src/force/grouprepulsionforce.cpp
	src/force/randomforce.cpp

	# waypointplanner
	src/waypointplanner/waypointplanner.cpp
	src/waypointplanner/individualwaypointplanner.cpp
	src/waypointplanner/queueingplanner.cpp
	src/waypointplanner/shoppingplanner.cpp
	src/waypointplanner/groupwaypointplanner.cpp
)


set(MOC_FILES
	include/pedsim_simulator/config.h
	include/pedsim_simulator/scene.h
	include/pedsim_simulator/agentstatemachine.h

	include/pedsim_simulator/element/scenarioelement.h
	include/pedsim_simulator/element/agent.h
	include/pedsim_simulator/element/agentcluster.h
	include/pedsim_simulator/element/agentgroup.h
	include/pedsim_simulator/element/attractionarea.h
	include/pedsim_simulator/element/obstacle.h
	include/pedsim_simulator/element/waypoint.h
	include/pedsim_simulator/element/areawaypoint.h
	include/pedsim_simulator/element/waitingqueue.h
	include/pedsim_simulator/element/queueingwaypoint.h

	include/pedsim_simulator/force/force.h
	include/pedsim_simulator/force/randomforce.h
	include/pedsim_simulator/force/groupgazeforce.h
	include/pedsim_simulator/force/groupcoherenceforce.h
	include/pedsim_simulator/force/grouprepulsionforce.h
	include/pedsim_simulator/force/alongwallforce.h

	include/pedsim_simulator/waypointplanner/waypointplanner.h
	include/pedsim_simulator/waypointplanner/individualwaypointplanner.h
	include/pedsim_simulator/waypointplanner/groupwaypointplanner.h
	include/pedsim_simulator/waypointplanner/shoppingplanner.h
	include/pedsim_simulator/waypointplanner/queueingplanner.h
)
qt5_wrap_cpp(MOC_SRCS_UI ${MOC_FILES})

set(EXECUTABLE_NAME ${PROJECT_NAME})
add_executable(${EXECUTABLE_NAME} ${SOURCES} ${MOC_SRCS_UI})
add_dependencies(${EXECUTABLE_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(${EXECUTABLE_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${EXECUTABLE_NAME}
  ${Qt5Widgets_LIBRARIES} ${BOOST_LIBRARIES} ${catkin_LIBRARIES}
)

add_executable(simulate_diff_drive_robot src/simulate_diff_drive_robot.cpp)
add_dependencies(simulate_diff_drive_robot ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulate_diff_drive_robot ${BOOST_LIBRARIES} ${catkin_LIBRARIES})

install(
  TARGETS
    ${EXECUTABLE_NAME}
    simulate_diff_drive_robot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Unit Tests
