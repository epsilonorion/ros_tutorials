cmake_minimum_required(VERSION 2.8.3)
project(action_tut)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  std_msgs
  message_generation
  )

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Example.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(example_action_server src/example_action_server.cpp)
add_dependencies(example_action_server example_action_gencpp)
target_link_libraries(
  example_action_server
  ${catkin_LIBRARIES}
)

add_executable(example_action_client src/example_action_client.cpp)
add_dependencies(example_action_client example_action_gencpp)
target_link_libraries(
  example_action_client
  ${catkin_LIBRARIES}
)
