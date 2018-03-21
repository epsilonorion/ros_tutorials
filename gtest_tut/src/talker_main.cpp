#include "gtest_tut/talker.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "talker");

  ros_tutorials::Talker talker;

  ros::Rate loop_rate(5);

  while(ros::ok())
  {
    talker.talk();
    loop_rate.sleep();
  }

  return 0;
}
