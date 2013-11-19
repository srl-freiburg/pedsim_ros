cmake_minimum_required(VERSION 2.8.3)
project(pedsim_srvs)


set( MESSAGE_DEPENDENCIES
    pedsim_msgs
    std_msgs
)

find_package(catkin REQUIRED 
  COMPONENTS
  message_generation
  ${MESSAGE_DEPENDENCIES}
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


# the services
add_service_files( DIRECTORY srv
  FILES
  SetAgentState.srv
  GetAgentState.srv
  SetAllAgentsState.srv
  GetAllAgentsState.srv
)

generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

#add_dependencies(some_target ${PROJECT_NAME}_gencpp)

catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES})