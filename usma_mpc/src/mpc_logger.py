#!/usr/bin/env python

# Import required Python code.
import cv2
import roslib
import rospy
import yaml
import tf
import message_filters
from geometry_msgs.msg import Twist, PoseStamped

class mpc_logger():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		param_data_root = rospy.get_param('~data_root')
		str_param_data_root = str(param_data_root)
		print "mpc_logger:: data_root:= " + str_param_data_root

		param_trial = rospy.get_param('~trial')
		str_trial = str(param_trial).zfill(3)
		print "mpc_logger::     trial:= " + str_trial

		exp_date = rospy.get_param('~date')
		str_date = str(exp_date)
		print "mpc_logger::      date:= " +str_date
		
		self.matlab_prefix = rospy.get_param("~matlab_prefix","mpc_log_")
		self.file_loggerfilename = ("{0}/{1}/{2}/{3}_{2}.m").format(str_param_data_root, str_date, str_trial, self.matlab_prefix)
		print("mpc_logger:: file_name:=" + self.file_loggerfilename)
		self.file_logger = open(self.file_loggerfilename, 'w')

		mpc_cmd_vel_topic = rospy.get_param('~mpc_cmd_vel_topic')
		uav_pose_topic = rospy.get_param('~uav_pose_topic')
		uav_des_pose_topic = rospy.get_param('~uav_des_pose_topic')

		self.cmd_vel_index = 1;
		self.uav_pose_index = 1;
		self.uav_des_pose_index = 1;

		self.cmd_vel_sub = rospy.Subscriber(mpc_cmd_vel_topic,Twist,self.cmd_vel_callback)
		self.uav_pose_sub = rospy.Subscriber(uav_pose_topic,PoseStamped,self.uav_pose_callback)
		self.uav_des_pose_sub = rospy.Subscriber(uav_des_pose_topic,PoseStamped,self.uav_des_pose_callback)

		self.des_pos_msg_seq = 0
		self.des_pos_msg_time = 0

		# <param name="/uav_des_pose_topic"   value="$(arg uav_des_pose_topic)" />
		# <param name="/uav_pose_topic"       value="$(arg uav_pose_topic)" />

		while not rospy.is_shutdown():
			rospy.spin()

	def cmd_vel_callback(self,msg):
		# seconds = rospy.get_time()
		# msg_time = rospy.Time.now()
		msg_time = rospy.get_time()

		# print(("{}.cmd_vel.time({},1) = {:06.8f};\n").format(self.matlab_prefix, self.cmd_vel_index, msg_time))
		# print(("{}.cmd_vel.angular({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.matlab_prefix, self.cmd_vel_index, msg.angular.x, msg.angular.y, msg.angular.z))
		# print(("{}.cmd_vel.linear({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.matlab_prefix, self.cmd_vel_index, msg.linear.x, msg.linear.y, msg.linear.z))
		self.file_logger.write(("{}.cmd_vel.time({},1) = {:06.8f};\n").format(self.matlab_prefix, self.cmd_vel_index, msg_time))
		self.file_logger.write(("{}.cmd_vel.linear({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.matlab_prefix, self.cmd_vel_index, msg.linear.x, msg.linear.y, msg.linear.z))
		self.file_logger.write(("{}.cmd_vel.angular({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.matlab_prefix, self.cmd_vel_index, msg.angular.x, msg.angular.y, msg.angular.z))
		self.cmd_vel_index += 1

	def uav_pose_callback(self,msg):
		# seconds = rospy.get_time()
		# msg_time = rospy.Time.now()
		msg_time = rospy.get_time()

		# print("pose: ")
		# print("  position: ")
		# print(("    x: {}").format(msg.pose.position.x))
		# print(("    y: {}").format(msg.pose.position.y))
		# print(("    z: {}").format(msg.pose.position.z))
		# print("  orientation: ")
		# print(("    x: {}").format(msg.pose.orientation.x))
		# print(("    y: {}").format(msg.pose.orientation.y))
		# print(("    z: {}").format(msg.pose.orientation.z))
		# print(("    w: {}").format(msg.pose.orientation.w))

		uav_position = msg.pose.position
		uav_orientation = msg.pose.orientation

		self.file_logger.write(("{}.uav_pose.time({},1) = {:06.8f};\n").format(self.matlab_prefix, self.uav_pose_index, msg_time))
		self.file_logger.write(("{}.uav_pose.position({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.matlab_prefix, self.uav_pose_index, uav_position.x, uav_position.y, uav_position.z))
		self.file_logger.write(("{}.uav_pose.orientation({},:) = [{:06.8f} {:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.matlab_prefix, self.uav_pose_index, uav_orientation.x, uav_orientation.y, uav_orientation.z, uav_orientation.w))
		self.uav_pose_index += 1

	def uav_des_pose_callback(self,msg):
		# seconds = rospy.get_time()
		# msg_time = rospy.Time.now()
		msg_time = rospy.get_time()

		uav_position = msg.pose.position
		uav_orientation = msg.pose.orientation

		self.file_logger.write(("{}.uav_des_pose.time({},1) = {:06.8f};\n").format(self.matlab_prefix, self.uav_pose_index, msg_time))
		self.file_logger.write(("{}.uav_des_pose.position({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.matlab_prefix, self.uav_pose_index, uav_position.x, uav_position.y, uav_position.z))
		self.file_logger.write(("{}.uav_des_pose.orientation({},:) = [{:06.8f} {:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.matlab_prefix, self.uav_pose_index, uav_orientation.x, uav_orientation.y, uav_orientation.z, uav_orientation.w))
		self.uav_des_pose_index += 1




if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('mpc_logger')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        log = mpc_logger()
    except rospy.ROSInterruptException: pass
