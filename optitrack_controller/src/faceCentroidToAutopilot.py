#!/usr/bin/env python

# Import required Python code.
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3, Twist #TwistStamped
from os.path import expanduser
home = expanduser("~")
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

    def facetracker_centroid_cb(self,msg):
        self.face_centroid_msg = msg
        self.face_centroid_msg_time = rospy.get_time()
        dx = self.param_image_Cx - self.face_centroid_msg.x # This contributes to yawing
        dy = self.param_image_Cy - self.face_centroid_msg.y # this contributes to altitude

        cmd_linear = Vector3(0, 0, self.Kz*dy)
        cmd_angular = Vector3(0, 0, self.Kyaw*dx)
        self.face_cmd_vel_msg = Twist(cmd_linear, cmd_angular)
        self.face_cmd_vel_pub.publish(self.face_cmd_vel_msg)

        if(self.logging):
            # print(self.face_centroid_topic + ' : {} {} {}' ).format(self.face_centroid_msg.x, self.face_centroid_msg.y, self.face_centroid_msg.z)
            self.face_centroid_counter += 1
            self.data_logger.write(("facetoCmd.face.centroid_msg.time({},1) = {};\n").format(self.face_centroid_counter, self.face_centroid_msg_time))
            self.data_logger.write(("facetoCmd.face.centroid_msg.linear({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.face_centroid_counter, self.face_centroid_msg.x, self.face_centroid_msg.y, self.face_centroid_msg.z))
            self.data_logger.write(("facetoCmd.face.centroid_msg.dxdy({},:) = [{:06.8f} {:06.8f} ];\n").format(self.face_centroid_counter, dx, dy))
            self.face_feedback_counter += 1
            self.data_logger.write(("facetoCmd.face.feedback.time({},1) = {:06.8f};\n").format(self.face_feedback_counter, rospy.get_time()))
            self.data_logger.write(("facetoCmd.face.feedback.yaw({},1) = {:06.8f};\n").format(self.face_feedback_counter, self.Kyaw*dx))
            self.data_logger.write(("facetoCmd.face.feedback.z({},1) = {:06.8f};\n\n").format(self.face_feedback_counter, self.Kz*dy))

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('faceCentroidToAutopilot')
    try:
        fta = faceCentroidToAutopilot()
    except rospy.ROSInterruptException: pass
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        rate.sleep()

    fta.data_logger.close()
# end main