################################################################################
# Author: Joshua Weaver
# Date: 3-16-18
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(gtest_tut)


################################################################################
# Find necessary ros packages needed for gtest_tut
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(GTest REQUIRED)

################################################################################
# Setup catkin package to setup ros dependencies.
################################################################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gtest_tut
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS} # Added for gtest
)

################################################################################
# Build listener application
################################################################################
SET(LSRC
  src/listener.cpp
  src/listener_main.cpp
)

add_executable(gtest_listener ${LSRC})

target_link_libraries(gtest_listener
  ${catkin_LIBRARIES}
)

################################################################################
# Build talker application
################################################################################
SET(TSRC
  src/talker.cpp
  src/talker_main.cpp
)

add_executable(gtest_talker ${TSRC})

target_link_libraries(gtest_talker
  ${catkin_LIBRARIES}
)

################################################################################
# Build test for talker
################################################################################
catkin_add_gtest(gtest_talker_test test/test_talker.cpp src/talker.cpp)

target_link_libraries(gtest_talker_test
  ${catkin_LIBRARIES}
)

###############################################################
# Install executable
###############################################################
install(TARGETS gtest_listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS gtest_talker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

