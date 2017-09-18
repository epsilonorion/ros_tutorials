################################################################################
# Author: Joshua Weaver
# Date: 8-11-15
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(class_tut)
add_definitions(--std=c++11)

################################################################################
# Find necessary ros packages needed for class_demo
################################################################################
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
)

################################################################################
# Setup catkin package to setup ros dependencies.
################################################################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES class_demo
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Build class_demo executable
################################################################################
set(BINNAME class_tut)

FILE(GLOB SRC 
	src/*.cpp
)
add_executable(${BINNAME} ${SRC})

target_link_libraries(${BINNAME}
  ${catkin_LIBRARIES}
)

###############################################################
# Install executable
###############################################################
install(TARGETS ${BINNAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

