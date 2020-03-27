cmake_minimum_required(VERSION 3.5)
project(pedsim)

#------------------ Configuration ------------------#
option(SHALL_DEBUG "Enable debug features" OFF)
option(SHALL_PROFILE "Enable the code profiling feature" OFF)
option(CMAKE_VERBOSE_MAKEFILE "Full compiler output" ON)


#----------------- Compiler Flags ------------------#
add_definitions(-Wall -Wunused -std=c++0x -pipe)

#debugging
if(SHALL_DEBUG)
  message("Debugging activated")
  add_definitions(-O0 -DDEBUG -ggdb -g3 -rdynamic)
else(SHALL_DEBUG)
  # message("Debugging deactivated")
  add_definitions(-Os)
endif(SHALL_DEBUG)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Boost REQUIRED)

set(dependencies
  rclcpp
)

include_directories(include)

set(SOURCES
  src/ped_agent.cpp
  src/ped_angle.cpp
  src/ped_obstacle.cpp
  src/ped_scene.cpp
  src/ped_tree.cpp
  src/ped_vector.cpp
  src/ped_waypoint.cpp
)

add_library(pedsim ${SOURCES})

target_link_libraries(pedsim
  ${BOOST_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  pedsim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(pedsim)
ament_export_dependencies(${dependencies})
ament_package()

