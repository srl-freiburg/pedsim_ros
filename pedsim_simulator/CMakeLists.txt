cmake_minimum_required(VERSION 3.5)
project(pedsim_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

set(PEDSIM_SIMULATOR_DEPENDENCIES
  rclcpp
  rclpy
  std_msgs
  pedsim
  pedsim_msgs
  pedsim_srvs
  pedsim_utils
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Qt5Widgets
  # cmake_modules
  # dynamic_reconfigure
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pedsim REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(pedsim_srvs REQUIRED)
find_package(pedsim_utils REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Qt5Widgets REQUIRED)

# dynamic reconfigure parameters
#generate_dynamic_reconfigure_options(config/PedsimSimulator.cfg)

include_directories(include)
include_directories(${Eigen_INCLUDE_DIRS})
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
	include/pedsim_simulator/config.hpp
	include/pedsim_simulator/scene.hpp
	include/pedsim_simulator/agentstatemachine.hpp

	include/pedsim_simulator/element/scenarioelement.hpp
	include/pedsim_simulator/element/agent.hpp
	include/pedsim_simulator/element/agentcluster.hpp
	include/pedsim_simulator/element/agentgroup.hpp
	include/pedsim_simulator/element/attractionarea.hpp
	include/pedsim_simulator/element/obstacle.hpp
	include/pedsim_simulator/element/waypoint.hpp
	include/pedsim_simulator/element/areawaypoint.hpp
	include/pedsim_simulator/element/waitingqueue.hpp
	include/pedsim_simulator/element/queueingwaypoint.hpp

	include/pedsim_simulator/force/force.hpp
	include/pedsim_simulator/force/randomforce.hpp
	include/pedsim_simulator/force/groupgazeforce.hpp
	include/pedsim_simulator/force/groupcoherenceforce.hpp
	include/pedsim_simulator/force/grouprepulsionforce.hpp
	include/pedsim_simulator/force/alongwallforce.hpp

	include/pedsim_simulator/waypointplanner/waypointplanner.hpp
	include/pedsim_simulator/waypointplanner/individualwaypointplanner.hpp
	include/pedsim_simulator/waypointplanner/groupwaypointplanner.hpp
	include/pedsim_simulator/waypointplanner/shoppingplanner.hpp
	include/pedsim_simulator/waypointplanner/queueingplanner.hpp
)
qt5_wrap_cpp(MOC_SRCS_UI ${MOC_FILES})

set(EXECUTABLE_NAME ${PROJECT_NAME})
add_executable(${EXECUTABLE_NAME} ${SOURCES} ${MOC_SRCS_UI})
ament_target_dependencies(${EXECUTABLE_NAME} ${PEDSIM_SIMULATOR_DEPENDENCIES})

#add_executable(simulate_diff_drive_robot src/simulate_diff_drive_robot.cpp)
#ament_target_dependencies(simulate_diff_drive_robot ${PEDSIM_SIMULATOR_DEPENDENCIES})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scenarios DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${EXECUTABLE_NAME}
  #simulate_diff_drive_robot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
