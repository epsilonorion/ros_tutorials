cmake_minimum_required(VERSION 2.8.3)
project(opencv_tut)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp sensor_msgs std_msgs message_generation)
find_package(OpenCV REQUIRED)

#generate_messages(
#  DEPENDENCIES
#  opencv_tut
#)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES opencv_tut
#  CATKIN_DEPENDS cv_bridge image_transport opencv2 roscpp sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

add_executable(invertColor src/invertColor.cpp)
target_link_libraries(invertColor ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(circleImage src/circleImage.cpp)
target_link_libraries(circleImage ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(simpleCanny src/simpleCanny.cpp)
target_link_libraries(simpleCanny ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(findCircle src/findCircle.cpp)
target_link_libraries(findCircle ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(imageTracking src/imageTracking.cpp)
target_link_libraries(imageTracking ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(findColor src/findColor.cpp)
target_link_libraries(findColor ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
