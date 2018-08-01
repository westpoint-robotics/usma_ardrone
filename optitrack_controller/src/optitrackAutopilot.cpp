//ROS Communications
#include <ros/ros.h>
	// Messages
	#include <ardrone_autonomy/Navdata.h>
	#include <std_msgs/Empty.h>
	#include <geometry_msgs/Twist.h>
	#include <nav_msgs/Odometry.h>
	#include <tf/transform_listener.h>

class optitrackAutopilot
{
	private:
		// ros::Subscriber HastShutDown_sub;
		// hast::flag Kill_msg;
		std_msgs::Empty Null_msg;
		// geometry_msgs::Twist DroneCmd_dr_msg;

	public:
		ros::NodeHandle n;

	optitrackAutopilot()
	{
		/*--------- Initialize ROS Communication & Variables ------------- */
		/*-----  Publishers and Subscribers */
		// HastShutDown_sub= n.subscribe("/hast/shutdown",   10,  &shutdownListener::nodeShutDown, this);
		ROS_INFO("optitrackAutopilot Constructed");
	}

	// void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	// {
	// 	if(ShutDown->flag)
	// 		{ros::shutdown();}
	// }

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "optitrackAutopilot");
	optitrackAutopilot oA;
	ros::spin();
	return 0;
}
