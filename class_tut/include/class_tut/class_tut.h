/**
 * @author Joshua Weaver
 * @date 6/22/17
 *
 * @brief Define a simple object class for an example of use within ROS
 *
 * A simple class to be used within ROS. Class is used to explain more complex
 * object oriented uses within ROS which is more similar to real applications
 * for projects than the listener and talker tutorials.
 */
#pragma once

#include <ros/ros.h>
#include "std_msgs/String.h"

namespace ros_tutorials
{
class ClassTut
{
public:
	ClassTut();
	~ClassTut();

private:
	ros::NodeHandle nh;		/// Nodehandle for ROS Node
	ros::NodeHandle pnh;	/// Private nodehandle to access Node/namespace specfics

	/// All subscriptions under node
	struct Subscriptions
	{
		ros::Subscriber listener;
	} mSubscriptions;

	/// All publishers under node
	struct Publishers
	{
		ros::Publisher talker;
	} mPublishers;

	/// Parameter to hold message for sending
	std::string mMessageToSend;

  /** Callback that handles receipt of listener messages
   *
   * Simple callback
   *
   * @param msg ROS message holding std_msgs String with data field
   */
	void listenerCallback(const std_msgs::String::ConstPtr& msg);

  /** Function to hold run loop of application
   *
   * Holds while loop with ROS sleep to handle main loop of application.
   */
	void runNode();
};
}
