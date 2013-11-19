cmake_minimum_required(VERSION 2.8.3)
project(pedsim_srvs)


set( MESSAGE_DEPENDENCIES
    pedsim_msgs
    roscpp
    rospy
    std_srvs
)

find_package(catkin REQUIRED 
  COMPONENTS
  message_generation
  ${MESSAGE_DEPENDENCIES}
)


# the services
add_service_files( DIRECTORY srv
  FILES
  SetAgentState.srv
  GetAgentState.srv
  SetAllAgentsState.srv
  GetAllAgentsState.srv
)

generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})



catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES})
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(pedsim_srvs
#   src/${PROJECT_NAME}/pedsim_srvs.cpp
# )

## Declare a cpp executable
# add_executable(pedsim_srvs_node src/pedsim_srvs_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pedsim_srvs_node pedsim_srvs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(pedsim_srvs_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pedsim_srvs pedsim_srvs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pedsim_srvs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
