#!/usr/bin/env python

# Import required Python code.
import sys
import rospy
import cv2
import math
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3, Twist #TwistStamped
from geometry_msgs.msg import Quaternion, Pose, Point
from os.path import expanduser
import time

class faceCentroidToAutopilot():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # check to see if data should be logged:
        self.logging = rospy.get_param("~logging",False) # false by default, in case operator doesn't know to create data folder for logging
        if(self.logging):
            self.exp_run = rospy.get_param("~run","002")
            self.exp_date = rospy.get_param("~date","20180917")
            self.mfile_variable = 'facetoCmd'
            self.data_logger_filename = ('/home/benjamin/ros/data/{0:0>3}/{1:0>3}/facetoCmd_{1:0>3}.m').format(self.exp_date, self.exp_run)
            self.data_logger = open(self.data_logger_filename, 'w')
            self.data_logger.write(("%%filename: {} \n\n").format(self.data_logger_filename))

        # wait for permission to track faces
        self.face_permission_topic = rospy.get_param("~face_permission_topic","/face_detector/permission")
        self.face_permission_sub = rospy.Subscriber(self.face_permission_topic,Empty,self.facetracker_permission_cb)
        self.face_permission_bool = False
        self.face_permission_time = 0

        #subscribe to face tracker centroid
        self.face_centroid_topic = rospy.get_param("~face_centroid_topic","/face_detector/centroid")
        self.face_centroid_sub = rospy.Subscriber(self.face_centroid_topic,Vector3,self.facetracker_centroid_cb)
        self.face_centroid_msg = Vector3(0, 0, 0)
        self.face_centroid_msg_time = None
        self.face_centroid_counter = 0
        
        self.param_image_width = rospy.get_param("ardrone/front/params/image_width","640")
        self.param_image_height = rospy.get_param("ardrone/front/params/image_height","360")
        self.param_image_Cx = rospy.get_param("ardrone/front/params/Cx","320")
        self.param_image_Cy = rospy.get_param("ardrone/front/params/Cy","180")

        # publish either the mocap cmd_vel or the face centroid based cmd_vel
        self.face_cmd_vel_topic = rospy.get_param("~face_cmd_vel_topic","/ardrone/face/cmd_vel")
        self.face_cmd_vel_pub = rospy.Publisher(self.face_cmd_vel_topic,Twist, queue_size=1)  
        self.face_cmd_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.face_cmd_vel_counter = 0

        # face tracker feedback commands
        self.face_feedback_counter = 0
        self.Kyaw = rospy.get_param("~Kyaw",0.005) # proportional gain for yaw feedback from image
        self.Kz = rospy.get_param("~Kz",0.01) # proportional gain for altitude feedback from image

        # face_desired_pose_msg
        self.face_desired_pose_counter = 0
        self.face_desired_pose_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        # self.face_desired_pose_msg = Pose
        self.face_desired_pose_point = Vector3(0, 0, 0)
        self.face_desired_pose_euler = Vector3(0, 0, 0)
        # self.face_desired_pose_point.x = 0
        # self.face_desired_pose_point.y = 0
        # self.face_desired_pose_point.z = 0
        # self.face_desired_pose_q = Quaternion
        self.face_pose_topic = rospy.get_param("~face_pose_topic","/ardrone/face/pose_desired")
        self.face_pose_pub = rospy.Publisher(self.face_pose_topic,Twist, queue_size=1)  

    def facetracker_permission_cb(self, msg):
        self.face_permission_time = rospy.get_time()
        self.face_permission_bool = True
        print("Received permission to track faces")

        if(self.logging):
            self.data_logger.write(("facetoCmd.permission.time = {:06.8f};\n").format(self.face_permission_time))

    def publish_face_desired_pose_msg(self, dx, dy):
        # qx = ax * sin(angle/2)
        # qy = ay * sin(angle/2)
        # qz = az * sin(angle/2)
        # qw = cos(angle/2)
        # ardrone FOV 92 degrees, 640 pixles :: 0.14375 degrees/pixel :: 0.00251 rads/pixel
        # self.face_desired_pose_q.x = 0
        # self.face_desired_pose_q.y = 0
        # self.face_desired_pose_q.z = math.sin(angle/2)
        # self.face_desired_pose_q.w = math.cos(angle/2)
        angle = dx*0.002 #reduced to slow down angular action
        # angle = dx*0.00251 based on view angle vs pixel pitch
        self.face_desired_pose_point = Vector3(0, 0, dy*0.002)
        self.face_desired_pose_euler = Vector3(0, 0, dx*0.002)
        self.face_desired_pose_msg = Twist(self.face_desired_pose_point, self.face_desired_pose_euler)
        if(self.logging):
            self.face_desired_pose_counter += 1
            self.data_logger.write(("facetoCmd.face.face_desired_pose.time({},1) = {:06.8f};\n").format(self.face_desired_pose_counter, rospy.get_time()))
            self.data_logger.write(("facetoCmd.face.face_desired_pose.point({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.face_desired_pose_counter, self.face_desired_pose_point.x, self.face_desired_pose_point.y, self.face_desired_pose_point.z))
            self.data_logger.write(("facetoCmd.face.face_desired_pose.euler({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.face_desired_pose_counter, self.face_desired_pose_euler.x, self.face_desired_pose_euler.y, self.face_desired_pose_euler.z))
        self.face_pose_pub.publish(self.face_desired_pose_msg)


    def facetracker_centroid_cb(self,msg):
        self.face_centroid_msg = msg
        self.face_centroid_msg_time = rospy.get_time()
        dx = self.param_image_Cx - self.face_centroid_msg.x # This contributes to yawing
        dy = self.param_image_Cy - self.face_centroid_msg.y # this contributes to altitude

        cmd_z = self.Kz*dy
        cmd_yaw = self.Kyaw*dx
        cmd_linear = Vector3(0, 0, cmd_z)
        cmd_angular = Vector3(0, 0, cmd_yaw)
        self.face_cmd_vel_msg = Twist(cmd_linear, cmd_angular)
        self.face_cmd_vel_pub.publish(self.face_cmd_vel_msg)
        if (self.face_permission_bool):
            self.publish_face_desired_pose_msg(dx, dy)

        if(self.logging):
            # print(self.face_centroid_topic + ' : {} {} {}' ).format(self.face_centroid_msg.x, self.face_centroid_msg.y, self.face_centroid_msg.z)
            self.face_centroid_counter += 1
            self.data_logger.write(("facetoCmd.face.centroid_msg.time({},1) = {};\n").format(self.face_centroid_counter, self.face_centroid_msg_time))
            self.data_logger.write(("facetoCmd.face.centroid_msg.linear({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.face_centroid_counter, self.face_centroid_msg.x, self.face_centroid_msg.y, self.face_centroid_msg.z))
            self.data_logger.write(("facetoCmd.face.centroid_msg.dxdy({},:) = [{:06.8f} {:06.8f} ];\n").format(self.face_centroid_counter, dx, dy))
            self.face_feedback_counter += 1
            self.data_logger.write(("facetoCmd.face.feedback.time({},1) = {:06.8f};\n").format(self.face_feedback_counter, rospy.get_time()))
            self.data_logger.write(("facetoCmd.face.feedback.yaw({},1) = {:06.8f};\n").format(self.face_feedback_counter, cmd_yaw))
            self.data_logger.write(("facetoCmd.face.feedback.z({},1) = {:06.8f};\n\n").format(self.face_feedback_counter, cmd_z))

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('faceCentroidToAutopilot')
    try:
        facetoCmd = faceCentroidToAutopilot()
    except rospy.ROSInterruptException: pass
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        rate.sleep()

    facetoCmd.data_logger.close()
# end main