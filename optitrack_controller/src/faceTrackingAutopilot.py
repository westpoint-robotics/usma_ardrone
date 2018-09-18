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

class faceTrackingAutopilot():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # check to see if data should be logged:
        self.logging = rospy.get_param("~logging",False) # false by default, in case operator doesn't know to create data folder for logging
        if(self.logging):
            self.exp_run = rospy.get_param("~run","001")
            self.exp_date = rospy.get_param("~date","20180917")
            self.data_logger_filename = '/home/benjamin/ros/data/' + self.exp_date + '/' + self.exp_run + '/faceTrackingAutopilot_' + self.exp_run + '.m'
            print("faceTrackingAutopilot :: logger filename {}\n\n").format(self.data_logger_filename)
            self.data_logger = open(self.data_logger_filename, 'w')
            self.data_logger.write(("%%filename: {}").format(self.data_logger_filename))

        #subscribe to optitrack cmd_vel 
        self.mocap_cmd_vel_topic = rospy.get_param("~mocap_cmd_vel_topic","/usma_ardrone/cmd_vel")
        self.mocap_vel_sub = rospy.Subscriber(self.mocap_cmd_vel_topic,Twist,self.mocap_cmd_vel_cb)
        self.mocap_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.mocap_vel_msg_time = None
        self.mocap_vel_counter = 0

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
        self.switched_cmd_vel_topic = rospy.get_param("~switched_cmd_vel_topic","/ardrone/switched/cmd_vel")
        self.switched_cmd_vel_pub = rospy.Publisher(self.switched_cmd_vel_topic,Twist, queue_size=1)  
        self.switched_cmd_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.switched_cmd_vel_counter = 0

        # face tracker feedback commands
        self.face_feedback_counter = 0
        self.Kyaw = rospy.get_param("~Kyaw","0.1") # proportional gain for yaw feedback from image
        self.Kz = rospy.get_param("~Kz","0.1") # proportional gain for altitude feedback from image


    def publish_switched_cmd_vel_msg(self):
        self.switched_cmd_vel_counter += 1
        self.data_logger.write(("fta.face.switched_cmd_vel_msg.time({},1) = {};\n").format(self.mocap_vel_counter, rospy.get_time()))
        self.data_logger.write(("fta.mocap.switched_cmd_vel_msg.linear(:,{}) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.mocap_vel_counter, self.mocap_vel_msg.linear.x, self.mocap_vel_msg.linear.y, fta.mocap_vel_msg.linear.z))
        self.data_logger.write(("fta.mocap.switched_cmd_vel_msg.angular(:,{}) = [{:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.mocap_vel_counter, self.mocap_vel_msg.angular.x, self.mocap_vel_msg.angular.y, fta.mocap_vel_msg.angular.z))
        self.switched_cmd_vel_pub.publish(self.switched_cmd_vel_msg)


    def mocap_cmd_vel_cb(self,msg):
        self.mocap_vel_counter += 1
        self.mocap_vel_msg = msg
        # print(self.mocap_cmd_vel_topic + '/linear : {} {} {}' ).format(self.mocap_vel_msg.linear.x, self.mocap_vel_msg.linear.y, self.mocap_vel_msg.linear.z)
        # print(self.mocap_cmd_vel_topic + '/angular : {} {} {}').format(self.mocap_vel_msg.angular.x, self.mocap_vel_msg.angular.y, self.mocap_vel_msg.angular.z)
        self.mocap_vel_msg_time = rospy.get_time()
        if(self.logging):
            self.data_logger.write(("fta.mocap.vel_msg.time({},1) = {};\n").format(self.mocap_vel_counter, self.mocap_vel_msg_time))
            self.data_logger.write(("fta.mocap.vel_msg.linear(:,{}) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.mocap_vel_counter, self.mocap_vel_msg.linear.x, self.mocap_vel_msg.linear.y, self.mocap_vel_msg.linear.z))
            self.data_logger.write(("fta.mocap.vel_msg.angular(:,{}) = [{:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.mocap_vel_counter, self.mocap_vel_msg.angular.x, self.mocap_vel_msg.angular.y, self.mocap_vel_msg.angular.z))
        if(self.face_centroid_msg_time == None): 
            self.publish_switched_cmd_vel_msg()

    def facetracker_centroid_cb(self,msg):
        self.face_centroid_counter += 1
        self.face_centroid_msg = msg
        # print(self.face_centroid_topic + ' : {} {} {}' ).format(self.face_centroid_msg.x, self.face_centroid_msg.y, self.face_centroid_msg.z)
        self.face_centroid_msg_time = rospy.get_time()
        if(self.logging):
            self.data_logger.write(("fta.face.centroid_msg.time({},1) = {};\n").format(self.mocap_vel_counter, self.mocap_vel_msg_time))
            self.data_logger.write(("fta.face.centroid_msg.linear(:,{}) = [{:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.face_centroid_counter, self.face_centroid_msg.x, self.face_centroid_msg.y, self.face_centroid_msg.z))


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('faceTrackingAutopilot')
    try:
        fta = faceTrackingAutopilot()
    except rospy.ROSInterruptException: pass

    dummy = None
    rate = rospy.Rate(30) # 30hz
    fta_face_ctrl_time = 0.1 # how many seconds should the last face detection be used for uav control
    fta_wait_for_face = 4 # how long before giving up and going back to neutral
    while not rospy.is_shutdown():
        if (fta.face_centroid_msg_time == None): # do nothing until a face is seen
            dummy = None
        else: # let the face tracker and time switcher choose
            # print ('rospy.get_time() - fta.face_centroid_msg_time = {}').format(rospy.get_time() - fta.face_centroid_msg_time)
            # print('now = {}, fta.face_centroid_msg_time = {}').format(rospy.get_time(), fta.face_centroid_msg_time)
            time_since_last_face = rospy.get_time() - fta.face_centroid_msg_time
            # print('time since last face was seen: {}').format(time_since_last_face) 
            if (time_since_last_face<=fta_face_ctrl_time):
                # Then calculate cmd_vel based on faces
                dx = fta.param_image_Cx - fta.face_centroid_msg.x # This contributes to yawing
                dy = fta.param_image_Cy - fta.face_centroid_msg.y # this contributes to altitude

                # use mocap to keep uav in center of workspace, let camera control altitude and yaw-rate
                cmd_linear = Vector3(fta.mocap_vel_msg.linear.x, fta.mocap_vel_msg.linear.y, fta.Kz*dy)
                cmd_angular = Vector3(0, 0, fta.Kyaw*dx)

                fta.face_feedback_counter += 1
                fta.data_logger.write(("fta.face.feedback.time({},1) = {};\n").format(fta.face_feedback_counter, rospy.get_time()))
                fta.data_logger.write(("fta.face.feedback.yaw({},1) = {};\n").format(fta.face_feedback_counter, fta.Kyaw*dx))
                fta.data_logger.write(("fta.face.feedback.z({},1) = {};\n\n").format(fta.face_feedback_counter, fta.Kz*dy))
                fta.switched_cmd_vel_msg = Twist(cmd_linear, cmd_angular)
            elif(
                    time_since_last_face>fta_face_ctrl_time and 
                    time_since_last_face<=fta_wait_for_face
                ): # do nothing, maybe the face will come back
                fta.switched_cmd_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            else: # lost the face, go home
                fta.switched_cmd_vel_msg = fta.mocap_vel_msg

            #publish cmd_vel no matter what
            fta.publish_switched_cmd_vel_msg()
        rate.sleep()

    fta.data_logger.close()
# end main