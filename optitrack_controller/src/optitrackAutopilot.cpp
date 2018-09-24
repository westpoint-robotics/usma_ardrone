#include <math.h>
//ROS Communications
#include <ros/ros.h>
	// Messages
	#include <ardrone_autonomy/Navdata.h>
	#include <std_msgs/Empty.h>
	#include <geometry_msgs/Twist.h>
	#include <geometry_msgs/Pose.h>
	#include <geometry_msgs/Vector3.h>
	#include <geometry_msgs/PoseStamped.h>
	#include <nav_msgs/Odometry.h>
	#include <tf/transform_listener.h>
	#include <opencv/cv.h>

	// #include <hast/flag.h>
	#include <usma_plugins/flag.h>
	#include <usma_plugins/ardrone_pose.h>

class optitrackAutopilot
{
	private:
		/*---------  File Recorder ------------- */
			std::string s_filename, s_run, s_dotm, s_root, s_handle, s_date, s_user;
			std::FILE * pFile;

		// shutdown manager
		ros::Subscriber ShutDown_sub, TakeOff_sub;
			std_msgs::Empty Null_msg;
			std::string s_take_off_topic;
			bool took_off;

		// uav commands
		ros::Publisher uav_cmd_pub;
			geometry_msgs::Twist uav_cmd_msg;
			double YawRateCommand;
			std::string s_uav_cmd_topic;

		// face pose subscriber
		ros::Subscriber face_pose_sub;
			std::string s_face_pose_topic;
			int face_pose_counter;
			geometry_msgs::Twist face_pose_msg;
			double face_desired_angle;


		// mocap subscriber
		ros::Subscriber mocap_pose_sub;
			std::string s_mocap_pose_topic;
			geometry_msgs::Pose uav_mocap_pose_msg;
			geometry_msgs::PoseStamped uav_mocap_poseStamped_msg;
			usma_plugins::ardrone_pose uav_mocap_ardrone_pose;
			tf::Quaternion uav_qt;
			tf::Vector3 uav_vt;
			tf::Transform uav_TF;
			double uav_RPY[3];
		    tf::Matrix3x3 uav_R;
		    cv::Mat R_from_yaw;
		    int mocap_counter, cmd_counter;
		    double last_mocap_heading;

		// desired pose subscriber
		ros::Subscriber desired_pose_sub;
			std::string s_desired_pose_topic;
			// geometry_msgs::Pose uav_desired_pose_global_msg, uav_desired_pose_body_msg;
			// # position.x/.y/.z : meters
			// # heading : radians
			usma_plugins::ardrone_pose uav_desired_pose_global_msg, uav_desired_pose_body_msg;

		// PV variables
		double Kp, Kv, Kphi;
		double starting_time;

	public:
		ros::NodeHandle n;

	optitrackAutopilot()
	{
		/*---------  File Recorder ------------- */
		ros::param::get("~date",s_date );
		ros::param::get("~run",s_run );
		// ROS_INFO("optitrackAutopilot: date = %s", s_date.c_str());
		// ROS_INFO("optitrackAutopilot: run = %s", s_run.c_str());
		s_handle = "optAutopilot_";
		s_root = "/home/benjamin/ros/data/";
		s_dotm = ".m";
		s_filename = s_root + s_date + "/" + s_run + "/" + s_handle + s_run + s_dotm;
		// ROS_INFO("optitrackAutopilot: %s", s_filename.c_str());
		pFile = std::fopen (s_filename.c_str(),"w");

		/*--------- Initialize ROS Communication & Variables ------------- */
		/*-----  Publishers and Subscribers */
		ros::param::get("~uav_cmd_topic", s_uav_cmd_topic);
			uav_cmd_pub = n.advertise<geometry_msgs::Twist>	(s_uav_cmd_topic, 1);
			cmd_counter = 0;

		ros::param::get("~face_pose_topic", s_face_pose_topic);
			face_pose_sub = n.subscribe(s_face_pose_topic,   10,  &optitrackAutopilot::updateFacePose, this);
			face_pose_counter = 0;
			face_desired_angle = 0;

		ros::param::get("~uav_desired_pose_topic", s_desired_pose_topic);
			desired_pose_sub = n.subscribe(s_desired_pose_topic,   10,  &optitrackAutopilot::updateDesiredPose, this);

		ros::param::get("~mocap_pose_topic", s_mocap_pose_topic);
			mocap_pose_sub = n.subscribe(s_mocap_pose_topic,   10,  &optitrackAutopilot::updatePoseStamped, this);
			mocap_counter = 0;
			last_mocap_heading = 0;

		ros::param::get("~take_off_topic", s_take_off_topic);
			TakeOff_sub = n.subscribe(s_take_off_topic,   10,  &optitrackAutopilot::wait_for_takeoff, this);
			face_pose_counter = 0;
			face_desired_angle = 0;




		ros::param::get("~Kp", Kp);
		ros::param::get("~Kv", Kv);
		ros::param::get("~Kphi", Kphi);

		initDesiredPose();


		ROS_INFO("optitrackAutopilot Constructed");
	}


		
	void wait_for_takeoff(const std_msgs::Empty::ConstPtr& msg)
	{
		starting_time = ros::Time::now().toSec();
	}


	void updateFacePose(const geometry_msgs::Twist::ConstPtr& msg)
	{
		face_pose_msg.linear = msg->linear; // radians
		face_pose_msg.angular = msg->angular; // radians
		face_desired_angle = face_pose_msg.angular.z;

		face_pose_counter += 1;
		fprintf (pFile,"optAutopilot.face.time(%d,:)  = % -6.8f;\n", face_pose_counter, ros::Time::now().toSec());
		fprintf (pFile,"optAutopilot.face.angle(%d,:) = % -6.8f;\n", face_pose_counter, face_desired_angle);
		fprintf (pFile,"optAutopilot.face.uav_mocap_heading(%d,:)      = % -6.8f;\n", face_pose_counter, uav_mocap_ardrone_pose.heading);
		fprintf (pFile,"optAutopilot.face.desired_global_heading(%d,:) = % -6.8f;\n\n", face_pose_counter, uav_desired_pose_global_msg.heading);
		
		// ROS_INFO("Adding %6.4f radians to global heading error", face_desired_angle);
		uav_desired_pose_global_msg.heading = uav_mocap_ardrone_pose.heading + face_desired_angle;
	}


	void updatePoseStamped(const geometry_msgs::PoseStamped::ConstPtr& mocap_pose_msg)
	{
		uav_mocap_pose_msg.position = mocap_pose_msg->pose.position;

		uav_mocap_pose_msg.orientation = mocap_pose_msg->pose.orientation;
        
        tf::Quaternion qt ( 
        	uav_mocap_pose_msg.orientation.x, 
        	uav_mocap_pose_msg.orientation.y, 
        	uav_mocap_pose_msg.orientation.z, 
        	uav_mocap_pose_msg.orientation.w);
        tf::Vector3 vt ( 
        	uav_mocap_pose_msg.position.x, 
        	uav_mocap_pose_msg.position.y, 
        	uav_mocap_pose_msg.position.z);
        tf::Transform baseTF ( uav_qt, uav_vt );
        uav_vt = vt;
        uav_qt = qt;
        uav_TF = baseTF;
        tf::Matrix3x3 m(qt);
        uav_R = m;
	    m.getRPY(uav_RPY[0], uav_RPY[1], uav_RPY[2]);

		uav_mocap_ardrone_pose.position.x = mocap_pose_msg->pose.position.x;
		uav_mocap_ardrone_pose.position.y = mocap_pose_msg->pose.position.y;
		uav_mocap_ardrone_pose.position.z = mocap_pose_msg->pose.position.z;
		// uav_mocap_ardrone_pose.heading = uav_RPY[2];
		double x = uav_mocap_pose_msg.orientation.x;
		double y = uav_mocap_pose_msg.orientation.y;
		double z = uav_mocap_pose_msg.orientation.z;
		double w = uav_mocap_pose_msg.orientation.w;
		// double h0 = 2.0 * (w * x + y * z);
		// double h1 = 1.0 - 2.0 * (x * x + y * y);
		// uav_mocap_ardrone_pose.heading = atan2(h0,h1);

		double h3 = 2.0 * (w * z + x * y);
		double h4 = 1.0 - 2.0 * (y * y + z * z);

		last_mocap_heading = uav_mocap_ardrone_pose.heading;
		uav_mocap_ardrone_pose.heading = atan2(h3,h4);

		// if(last_mocap_heading <=-2.9 && atan2(h3,h4) >= 2.9)
		// {
		// 	uav_mocap_ardrone_pose.heading = atan2(h3,h4)
		// }

		// ROS_INFO("uav_mocap_ardrone_pose.heading = atan2(h0,h1) = %6.4f", uav_mocap_ardrone_pose.heading);


		mocap_counter += 1;
		fprintf (pFile,"optAutopilot.mocap_pose.time(%d,:) = % -6.8f;\n", mocap_counter, ros::Time::now().toSec());
		fprintf (pFile,"optAutopilot.mocap_pose.p(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", mocap_counter, 
																				uav_mocap_pose_msg.position.x, 
																				uav_mocap_pose_msg.position.y, 
																				uav_mocap_pose_msg.position.z);
		fprintf (pFile,"optAutopilot.mocap_pose.q(%d,:) = [% -6.8f % -6.8f % -6.8f % -6.8f];\n", mocap_counter, 
																				uav_mocap_pose_msg.orientation.x, 
																				uav_mocap_pose_msg.orientation.y, 
																				uav_mocap_pose_msg.orientation.z, 
																				uav_mocap_pose_msg.orientation.w);
		fprintf (pFile,"optAutopilot.mocap_pose.yaw(%d,:) = % -6.8f;\n\n", mocap_counter, uav_mocap_ardrone_pose.heading);
				// now compute UAV commands
			uav_Kp();
	}


	void updatePose(const geometry_msgs::Pose::ConstPtr& mocap_pose_msg)
	{
		uav_mocap_pose_msg.position = mocap_pose_msg->position;

		uav_mocap_pose_msg.orientation = mocap_pose_msg->orientation;
        
        tf::Quaternion qt ( 
        	uav_mocap_pose_msg.orientation.x, 
        	uav_mocap_pose_msg.orientation.y, 
        	uav_mocap_pose_msg.orientation.z, 
        	uav_mocap_pose_msg.orientation.w);
        tf::Vector3 vt ( 
        	uav_mocap_pose_msg.position.x, 
        	uav_mocap_pose_msg.position.y, 
        	uav_mocap_pose_msg.position.z);
        tf::Transform baseTF ( uav_qt, uav_vt );
        uav_vt = vt;
        uav_qt = qt;
        uav_TF = baseTF;
        tf::Matrix3x3 m(qt);
        uav_R = m;
	    m.getRPY(uav_RPY[0], uav_RPY[1], uav_RPY[2]);

		uav_mocap_ardrone_pose.position.x = mocap_pose_msg->position.x;
		uav_mocap_ardrone_pose.position.y = mocap_pose_msg->position.y;
		uav_mocap_ardrone_pose.position.z = mocap_pose_msg->position.z;


	    // def yaw_from_q(self, w, x, y, z):
	    //     t0 = +2.0 * (w * x + y * z)
	    //     t1 = +1.0 - 2.0 * (x * x + y * y)
	    //     X = math.degrees(math.atan2(t0, t1))
	    //     return X

		// double x = uav_mocap_pose_msg.orientation.x;
		// double y = uav_mocap_pose_msg.orientation.y;
		// double z = uav_mocap_pose_msg.orientation.z;
		// double w = uav_mocap_pose_msg.orientation.w;
		// double h0 = 2.0 * (w * x + y * z);
		// double h1 = 1.0 - 2.0 * (x * x + y * y);

		// uav_mocap_ardrone_pose.heading = atan2(h0,h1);
		// ROS_INFO("uav_mocap_ardrone_pose.heading = atan2(h0,h1) = %6.4f", uav_mocap_ardrone_pose.heading);

		// now compute UAV commands
			uav_Kp();
	}

	void initDesiredPose()
	{
		uav_desired_pose_body_msg.position.x = uav_desired_pose_global_msg.position.y = 0; // meters
		uav_desired_pose_body_msg.position.x = uav_desired_pose_global_msg.position.y = 0; // meters
		uav_desired_pose_body_msg.position.z = uav_desired_pose_global_msg.position.z = 1.5; // meters
		uav_desired_pose_body_msg.heading 	 = uav_desired_pose_global_msg.heading = 0; // radians
		R_from_yaw = (cv::Mat_<double>(3, 3) << 1, 0, 0,
												0, 1, 0,
												0, 0, 1);

	}

	void updateDesiredPose(const usma_plugins::ardrone_pose::ConstPtr& desired_pose_msg)
	{
		uav_desired_pose_global_msg.heading = desired_pose_msg->heading; // radians
		uav_desired_pose_global_msg.position = desired_pose_msg->position; // radians
	}

	void uav_Kp()
	{
		cv::Mat global_position_error = (cv::Mat_<double>(3, 1) <<  
			uav_desired_pose_global_msg.position.x - uav_mocap_ardrone_pose.position.x,
			uav_desired_pose_global_msg.position.y - uav_mocap_ardrone_pose.position.y,
			uav_desired_pose_global_msg.position.z - uav_mocap_ardrone_pose.position.z);
		double global_heading_error = uav_desired_pose_global_msg.heading - uav_mocap_ardrone_pose.heading;
        // ROS_INFO("uav_Kp global position and heading errors [x, y, z, Y] : [%2.3f, %2.3f, %2.3f, %2.4f]", 
		// global_position_error.at<double>(0,0),
		// global_position_error.at<double>(1,0),
		// global_position_error.at<double>(2,0),
        // 	global_heading_error);

		// convert global frame to uav body frame
		double cosyaw = cos(uav_RPY[2]);
		double sinyaw = sin(uav_RPY[2]);

		R_from_yaw = (cv::Mat_<double>(3, 3) <<  cosyaw, sinyaw, 0,
												-sinyaw, cosyaw, 0,
												 0, 0, 1);
		cv::Mat body_position_error = R_from_yaw * global_position_error;
        // ROS_INFO("uav_Kp body position and heading errors [x, y, z, Y] : [%2.3f, %2.3f, %2.3f, %2.4f]", 
        	// body_position_error.at<double>(0,0),
        	// body_position_error.at<double>(1,0),
        	// body_position_error.at<double>(2,0),
        // 	global_heading_error);


		uav_cmd_msg.linear.x = Kp*body_position_error.at<double>(0,0);
		uav_cmd_msg.linear.y = Kp*body_position_error.at<double>(1,0);
		uav_cmd_msg.linear.z = 2*Kp*body_position_error.at<double>(2,0);
		uav_cmd_msg.angular.x = 0;
		uav_cmd_msg.angular.y = 0;
		uav_cmd_msg.angular.z = -Kphi*global_heading_error;

		// cmdUAV(uav_cmd_msg);
		if ((ros::Time::now().toSec() - starting_time) < 5)
		{
			// DO nothing while uav takes off
			ROS_INFO("Waiting for UAV to takeoff");
		} else {
			uav_cmd_pub.publish(uav_cmd_msg);	
		}
		


		cmd_counter += 1;
		fprintf (pFile,"optAutopilot.cmd.time(%d,:) = % -6.8f;\n", cmd_counter, ros::Time::now().toSec());
		fprintf (pFile,"optAutopilot.cmd.desired_pose_global.yaw(%d,:) = % -6.8f;\n", cmd_counter, uav_desired_pose_global_msg.heading);
		fprintf (pFile,"optAutopilot.cmd.face_desired_angle(%d,:) = % -6.8f;\n", cmd_counter, face_desired_angle);
		fprintf (pFile,"optAutopilot.cmd.uav_mocap_ardrone_pose.heading(%d,:) = % -6.8f;\n", cmd_counter, uav_mocap_ardrone_pose.heading);
		fprintf (pFile,"optAutopilot.cmd.desired_pose_global.p(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", cmd_counter, 
																				uav_desired_pose_global_msg.position.x, 
																				uav_desired_pose_global_msg.position.y, 
																				uav_desired_pose_global_msg.position.z);
		fprintf (pFile,"optAutopilot.cmd.error_pose_global.yaw(%d,:) = % -6.8f;\n", cmd_counter, global_heading_error);
		fprintf (pFile,"optAutopilot.cmd.error_pose_global.p(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", cmd_counter, 
																				global_position_error.at<double>(0,0),
																				global_position_error.at<double>(1,0),
																				global_position_error.at<double>(2,0));
		fprintf (pFile,"optAutopilot.cmd.error_pose_body.yaw(%d,:) = % -6.8f;\n", cmd_counter, uav_desired_pose_global_msg.heading);
		fprintf (pFile,"optAutopilot.cmd.error_pose_body.p(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", cmd_counter, 
																				body_position_error.at<double>(0,0),
																				body_position_error.at<double>(1,0),
																				body_position_error.at<double>(2,0));
		fprintf (pFile,"optAutopilot.cmd.msg_linear(%d,:) = [% -6.8f % -6.8f % -6.8f];\n", cmd_counter, uav_cmd_msg.linear.x, uav_cmd_msg.linear.y, uav_cmd_msg.linear.z);
		fprintf (pFile,"optAutopilot.cmd.msg_angular(%d,:)= [% -6.8f % -6.8f % -6.8f];\n\n", cmd_counter, uav_cmd_msg.angular.x, uav_cmd_msg.angular.y, uav_cmd_msg.angular.z);





	}

	// void cmdUAV(geometry_msgs::Twist cmd_vel)
	// {
	// 	uav_cmd_pub.publish(cmd_vel);
	// 	// cmd_count += 1;
	// 	// fprintf (pFile,"\nuavCon.cmd.time(%d,:) = % -6.8f;\n", cmd_count, ros::Time::now().toSec());
	// 	// fprintf (pFile,"uavCon.cmd.linear(%d,:)  = [% -6.8f, % -6.8f, % -6.8f];\n", cmd_count, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);
	// 	// fprintf (pFile,"uavCon.cmd.angular(%d,:) = [% -6.8f, % -6.8f, % -6.8f];\n", cmd_count, cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
	// }



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
