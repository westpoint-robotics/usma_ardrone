//ROS Communications
#include <ros/ros.h>
	// Messages
	#include <ardrone_autonomy/Navdata.h>
	#include <std_msgs/Empty.h>
	#include <geometry_msgs/Twist.h>
	#include <geometry_msgs/Pose.h>
	#include <nav_msgs/Odometry.h>
	#include <tf/transform_listener.h>

	#include <usma_plugins/flag.h>

class optitrackAutopilot
{
	private:
		// shutdown manager
		ros::Subscriber ShutDown_sub;
			std_msgs::Empty Null_msg;

		// uav commands
		ros::Publisher uav_cmd_pub;
			geometry_msgs::Twist uav_cmd_msg;
			double YawRateCommand;
			std::string s_uav_cmd_topic;

		// mocap subscriber
		ros::Subscriber mocap_pose_sub;
			std::string s_mocap_pose_topic;
			geometry_msgs::Pose uav_mocap_pose;
			tf::Quaternion uav_qt;
			tf::Vector3 uav_vt;
			tf::Transform uav_TF;



	public:
		ros::NodeHandle n;

	optitrackAutopilot()
	{
		/*--------- Initialize ROS Communication & Variables ------------- */
		/*-----  Publishers and Subscribers */
		ROS_INFO("optitrackAutopilot Constructed");

		ros::param::get("~uav_cmd_topic", s_uavcmd_topic);
			DroneCmd_dr_pub 	= n.advertise<geometry_msgs::Twist>	(s_uavcmd_topic, 1);

		ros::param::get("~uav_cmd_topic", s_uavcmd_topic);
			mocap_pose_sub= n.subscribe(s_mocap_pose_topic,   10,  &optitrackAutopilot::updatePose, this);


	}

	void updatePose(const geometry_msgs::Pose::ConstPtr& mocap_pose_msg)
	{
		uav_mocap_pose_msg = mocap_pose_msg;
        
        uav_qt = tf::Quaternion qt ( 
        	uav_mocap_pose_msg.orientation.x, 
        	uav_mocap_pose_msg.orientation.y, 
        	uav_mocap_pose_msg.orientation.z, 
        	uav_mocap_pose_msg.orientation.w);
        uav_vt = tf::Vector3 vt ( 
        	uav_mocap_pose_msg.position.x, 
        	uav_mocap_pose_msg.position.y, 
        	uav_mocap_pose_msg.position.z);
        uav_TF = tf::Transform baseTF ( uav_qt, uav_vt );


	}

	void nodeShutDown(const usma_plugins::flag::ConstPtr& ShutDown)
	{
		if(ShutDown->flag)
			{ros::shutdown();}
	}

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "optitrackAutopilot");
	optitrackAutopilot oA;
	ros::spin();
	return 0;
}
