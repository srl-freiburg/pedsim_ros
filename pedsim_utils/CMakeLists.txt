cmake_minimum_required(VERSION 3.5)
project(pedsim_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  geometry_msgs
  rclcpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME}
  src/${PROJECT_NAME}/geometry.cpp
  src/${PROJECT_NAME}/pedsim_utils.cpp
)

ament_target_dependencies(${LIBRARY_NAME}
  ${dependencies}
)


install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${dependencies})

ament_package()
