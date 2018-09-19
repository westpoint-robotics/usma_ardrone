
# //ROS Communications
# #include <ros/ros.h>
# 	// Messages
# 	#include <ardrone_autonomy/Navdata.h>
# 	#include <std_msgs/Empty.h>

# 	std_msgs::Empty null;

# 	int main(int argc, char **argv)
# 	{
# 		ros::init(argc, argv, "uavCal");

# 		ros::NodeHandle n;
# 			/*-----  Publishers and Subscribers */
# 			ros::Publisher imuRecal_pub	= n.advertise<std_msgs::Empty>("/ardrone/imu_recalib", 1000);
# 			ros::Publisher flatTrim_pub	= n.advertise<std_msgs::Empty>("/ardrone/flattrim", 1000);
# 			ros::Duration(0.5).sleep(); // sleep for 'x' second(s).

# 			ros::spinOnce();

# 		ROS_INFO("Publishing Flat-Trim Command...");
# 			flatTrim_pub.publish(null);
# 			ros::Duration(3).sleep(); // sleep for 'x' second(s).
# 			ROS_INFO("...done");

# 		ROS_INFO("Publishing IMU reCal Command...");
# 			imuRecal_pub.publish(null);
# 			ros::Duration(3).sleep(); // sleep for 'x' second(s).
# 			ROS_INFO("...done");


# 		return 0;
# 	}


# <node pkg="rosservice" type="rosservice" name="togglecam" args="call /ardrone/flattrim"/>