cmake_minimum_required(VERSION 2.8.3)
project(smach_tut)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  action_tut
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES smach
  CATKIN_DEPENDS rospy smach smach_ros
#  DEPENDS system_lib
)

include_directories(#include
  ${catkin_INCLUDE_DIRS}
)
