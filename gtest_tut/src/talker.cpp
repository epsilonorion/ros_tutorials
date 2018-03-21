#include "gtest_tut/talker.h"

using namespace ros_tutorials;

Talker::Talker()
{
  _pub = _nh.advertise<std_msgs::String>("talker", 1000);
}

void Talker::talk()
{
  std_msgs::String msg;
  std::stringstream ss;

  ss<<"hello world";

  msg.data = ss.str();
  _pub.publish(msg);
}

int Talker::add(int a, int b)
{
  return a + b;
}
