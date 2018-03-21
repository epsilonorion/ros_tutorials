#include "gtest_tut/listener.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros_tutorials::Listener listener;

  ros::spin();

  return 0;
}
