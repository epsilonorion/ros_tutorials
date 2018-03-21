/**
 * @author Joshua Weaver
 * @date 8/11/15
 *
 * @brief Define a simple object class for an example of use within ROS
 *
 * A simple class to be used within ROS. Class is used to explain more complex
 * object oriented uses within ROS which is more similar to real applications
 * for projects than the listener and talker tutorials.
 */
 #include "class_tut/class_tut.h"

using namespace ros_tutorials;

ClassTut::ClassTut():
	pnh("~")
{
	/// Setup parameters
	pnh.param<std::string>("message", mMessageToSend, "not_set");

	/// Setup publishers
	mPublishers.talker = nh.advertise<std_msgs::String>( "talker", 10, true);

	/// Setup subscribers
	mSubscriptions.listener = nh.subscribe("listener", 10, &ClassTut::listenerCallback, this);

	ROS_INFO("Class tutorial initialized");

	runNode();
}

ClassTut::~ClassTut()
{

}

void ClassTut::listenerCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Received: %s", msg->data.c_str());
}

void ClassTut::runNode()
{
	ROS_INFO("Starting class tutorial node");

	/// Set loop rate of main node to 1 Hz
	ros::Rate loopRate(1);

	while(ros::ok())
	{
		/// Build ROS Message for talker publisher
		std_msgs::String msg;
		msg.data = mMessageToSend;

		mPublishers.talker.publish(msg);

		/// Spin ROS node at above loopRate
		ros::spinOnce();
		loopRate.sleep();
	}
}
