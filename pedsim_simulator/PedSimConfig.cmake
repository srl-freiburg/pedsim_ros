# PedSim Configuration
# A CMake configuration file for an already installed PedSim installation.
#
# CMake configuration author: Sven Wehner <mail@svenwehner.de>
# PedSim Manufacturer: http://pedsim.silmaril.org/
#
# Usage: 
#  1. Copy this file into the project's base folder.
#  2. Edit the following variables according to your installation:
#       PEDSIM_INCLUDE_DIR
#       PEDSIM_LIBRARY
#
# In your CMakeLists.txt file you can use the following code:
#------------------------ CMake snippet -------------------------------
# # Add PedSim to the project
# FIND_PACKAGE(PedSim REQUIRED)
# INCLUDE_DIRECTORIES(${PEDSIM_INCLUDE_DIR})
# ...
# TARGET_LINK_LIBRARIES(application_name ${PEDSIM_LIBRARIES})
#----------------------------------------------------------------------

# Configuration
# → directory which includes libPedSim's header files
SET(PEDSIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libpedsim)

# → PedSim's library
SET(PEDSIM_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libpedsim/libpedsim.so)


#------------------------ NOTHING TO DO BELOW THIS LINE ------------------------
SET(PEDSIM_FOUND TRUE)
SET(PEDSIM_LIBRARIES "${PEDSIM_LIBRARY}")

# Cache the results
# (you have to clean the cache if anything changes!)
MARK_AS_ADVANCED(
	PEDSIM_INCLUDE_DIR
	PEDSIM_LIBRARY
	PEDSIM_LIBRARIES
)

