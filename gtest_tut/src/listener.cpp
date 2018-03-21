#include "gtest_tut/listener.h"

using namespace ros_tutorials;

Listener::Listener()
{
  _sub = _nh.subscribe("talker", 1000, &Listener::listenerCallback, this);
}

void Listener::listenerCallback(const std_msgs::String & msg)
{
  ROS_INFO_STREAM("I heard from talker: "<< msg.data.c_str());
}
