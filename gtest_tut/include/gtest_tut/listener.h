/**
 * @author Joshua Weaver
 * @date 3/16/18
 *
 * @brief Define a simple listener class
 *
 * Define a simple listener class that may be used for testing gtest functions.
 * Class only holds a callback function for listening to messages.
 */
#pragma once

#include <ros/ros.h>
#include "std_msgs/String.h"

namespace ros_tutorials
{
class Listener{
public:
  ros::NodeHandle _nh;    /// Nodehandle for ROS node
  ros::Subscriber _sub;   /// Subscriber for std_msgs topic

  Listener();

  /** Callback that handles receipt of listener messages
   *
   * Simple callback
   *
   * @param msg ROS message holding std_msgs String with data field
   */
  void listenerCallback(const std_msgs::String & msg);
};
}
