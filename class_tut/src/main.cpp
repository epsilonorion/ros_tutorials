/**
 * @author Joshua Weaver
 * @date 8/11/15
 *
 * @brief Main file for starting ROS node on Class Tutorials
 */
 #include "class_tut/class_tut.h"

int main(int argc, char **argv)
{
	/// Initialize ROS Node
	ROS_INFO("Class tutorial initializing...");
	ros::init(argc, argv, "class_tut");

	/// Create class object
	ros_tutorials::ClassTut classTut;

	/// ROS Spin is only needed if the class object doesn't have a loop (which it 
	/// does) or if it is using a separately created thread to handle looping. 
	/// Leaving this here for reference.
	ros::Rate r(10.0);
	ros::spin();

	ROS_INFO("Class tutorial shutting down...");

	return 0;
}
