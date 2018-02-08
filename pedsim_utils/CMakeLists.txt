cmake_minimum_required(VERSION 2.8.3)
project(pedsim_utils)
add_compile_options(-Wall -Wunused -std=c++11)

set(PACKAGE_DEPS
  cmake_modules
  geometry_msgs
  pedsim_msgs
  roscpp
  rospy
)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPS})
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPS}
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS
    include
)

###########
## Build ##
###########
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME}
  src/${PROJECT_NAME}/geometry.cpp
  src/${PROJECT_NAME}/pedsim_utils.cpp
)

add_dependencies(${LIBRARY_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${LIBRARY_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pedsim_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
