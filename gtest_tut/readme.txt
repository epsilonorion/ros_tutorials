###############################################################################
Setting up gtest
###############################################################################
To use gtest for ROS on ubuntu, the following needs to be done to install gtest.

sudo apt-get install libgtest-dev
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib

###############################################################################
Building test
###############################################################################
All needed components for compiling a test are found in the CMakeLists.txt file.
Nothing is needed in package.xml.

To build the test, instead of running catkin_make at the root of the workspace, 
run catkin_make tests.

###############################################################################
Running
###############################################################################
To run the test, you treat it as a node. This means simply run roscore and then
run the test node.

###############################################################################
Resources
###############################################################################
The following links were used in creating these tests

https://github.com/google/googletest
http://wiki.ros.org/gtest
http://ysonggit.github.io/coding/2014/12/19/use-gtest-in-ros-program.html

A much larger project example, but very useful
https://github.com/ros/ros_comm/blob/ebd9e491e71947889eb81089306698775ab5d2a2/test/test_roscpp/test/src/subscribe_star.cpp#L123
