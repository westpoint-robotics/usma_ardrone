// Programming tools
#include <math.h>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iostream>

//Vision tools
// #include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <camera1394/SetCameraRegisters.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

//ROS Communications
#include <ros/ros.h>
	// Messages
	#include <std_msgs/Empty.h>
	#include <std_msgs/Bool.h>
	#include <geometry_msgs/Twist.h>

	std_msgs::Empty Null_msg;
	std_msgs::Bool Kill_msg;
	geometry_msgs::Twist DroneCmd_dr_msg;

	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "NodeKill");
		ros::NodeHandle n;
			/*-----  Publishers and Subscribers */
			std::string s_kill_topic, s_land_topic, s_cmd_topic;
			ros::param::get("~kill_topic", s_kill_topic);
			ros::param::get("~land_topic", s_land_topic);
			ros::param::get("~cmd_topic",  s_cmd_topic);

			ros::Publisher NodeKill		= n.advertise<std_msgs::Bool>(s_kill_topic, 1000);
			ros::Publisher DroneLand	= n.advertise<std_msgs::Empty>(s_land_topic, 1000);
			ros::Publisher DroneCmd_dr_pub 	= n.advertise<geometry_msgs::Twist>	(s_cmd_topic, 1000);

			DroneCmd_dr_msg.linear.x = 0.0;
			DroneCmd_dr_msg.linear.y = 0.0;
			DroneCmd_dr_msg.linear.z = 0.0;
			DroneCmd_dr_msg.angular.z = 0.0;
			DroneCmd_dr_msg.angular.x = 0.0;
			DroneCmd_dr_msg.angular.y = 0.0;
		while (ros::ok())
		{
			ROS_INFO("Ready to kill Nodes.\n");
			Kill_msg.data = true;
			std::cin.ignore();
			ROS_INFO("Killing Nodes...");
			ROS_INFO("Stopping Drone...");
			ROS_INFO("Landing Drone...");
			for(int b = 1; b < 15; ++b)
			{
				NodeKill.publish(Kill_msg);
				DroneCmd_dr_pub.publish(DroneCmd_dr_msg);
				DroneLand.publish(Null_msg);
				ros::spinOnce();
				ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
			}

			ROS_INFO("Resetting Killswitch...");
			for(int b = 1; b < 15; ++b)
			{
				Kill_msg.data = false;
				NodeKill.publish(Kill_msg);
				ros::spinOnce();
				ros::Duration(0.1).sleep(); // sleep for 'x' second(s).
			}
			ROS_INFO("...done\n");
			ros::shutdown();
		}

		return 0;
	}
