//ROS Communications
#include <ros/ros.h>
	// Messages
	#include <std_msgs/Empty.h>
	#include <std_msgs/Bool.h>


class shutdownListener
{
	private:
		ros::Subscriber ShutDown_sub;
		std_msgs::Bool Kill_msg;
		std_msgs::Empty Null_msg;

	public:
		ros::NodeHandle n;

	shutdownListener()
	{
		/*--------- Initialize ROS Communication & Variables ------------- */
		/*-----  Publishers and Subscribers */
		ShutDown_sub= n.subscribe("/usma/shutdown",   10,  &shutdownListener::nodeShutDown, this);
		ROS_INFO("waitForkill Listener Constructed");
	}

	void nodeShutDown(const std_msgs::Bool::ConstPtr& ShutDown)
	{
		if(ShutDown->data)
			{ros::shutdown();}
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waitForkill");
	shutdownListener tC;
	ros::spin();
	return 0;
}
