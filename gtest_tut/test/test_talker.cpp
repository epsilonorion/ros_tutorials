#include <climits>

#include <gtest/gtest.h>
#include "gtest_tut/talker.h"

// bad function:
// for example: how to deal with overflow?
int add(int a, int b){
  return a + b;
}

TEST(NumberCmpTest, ShouldPass){
  ASSERT_EQ(3, add(1,2));
}

TEST(NumberCmpTest, ShouldFail){
  ASSERT_EQ(INT_MAX, add(INT_MAX, 1));
}

TEST(TalkerTest, TalkerFunction){
  ros_tutorials::Talker talker;
  ASSERT_EQ(3, talker.add(1,2));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  // do not forget to init ros because this is also a node
  ros::init(argc, argv, "talker_tester");

  return RUN_ALL_TESTS();
}
