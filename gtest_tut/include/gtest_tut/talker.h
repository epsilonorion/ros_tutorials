/**
 * @author Joshua Weaver
 * @date 3/16/18
 *
 * @brief Define a simple talker class
 *
 * Define a simple talker class that may be used for testing gtest functions.
 * Class only holds a function for publishing a message and a function for 
 * adding variables.
 */
#pragma once

#include <sstream>

#include <ros/ros.h>
#include "std_msgs/String.h"

namespace ros_tutorials
{
class Talker{
public:
  ros::NodeHandle _nh;    /// Nodehandle for ROS node
  ros::Publisher _pub;    /// Publisher for std_msgs topic

  Talker();

  /** Handle transmit of std_msg string messages
   *
   * Simple function used to transmit a std_msg string messages.
   */
  void talk();

  /** Add two numbers and return result
   *
   * @param x First integer for addition
   * @param y Second integer for addition
   * @return Result of summing two integers
   */
  int add(int x, int y);
};
}
