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
        #subscribe to optitrack cmd_vel 
        self.mocap_cmd_vel_topic = rospy.get_param("~mocap_cmd_vel_topic","/usma_ardrone/cmd_vel")
        self.mocap_vel_sub = rospy.Subscriber(self.mocap_cmd_vel_topic,Twist,self.mocap_cmd_vel_cb)
        self.mocap_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        #subscribe to face tracker centroid
        self.face_centroid_topic = rospy.get_param("~face_centroid_topic","/face_detector/centroid")
        self.face_centroid_sub = rospy.Subscriber(self.face_centroid_topic,Vector3,self.facetracker_centroid_cb)
        self.face_centroid_msg = Vector3(0, 0, 0)
        self.face_centroid_msg_time = None
        
        # publish either the mocap cmd_vel or the face centroid based cmd_vel
        self.switched_cmd_vel_topic = rospy.get_param("~switched_cmd_vel_topic","/ardrone/switched/cmd_vel")
        self.switched_cmd_vel_pub = rospy.Publisher(self.switched_cmd_vel_topic,Twist, queue_size=1)  
        self.switched_cmd_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        self.param_image_width = rospy.get_param("ardrone/front/params/image_width","640")
        self.param_image_height = rospy.get_param("ardrone/front/params/image_height","360")
        self.param_image_Cx = rospy.get_param("ardrone/front/params/Cx","320")
        self.param_image_Cy = rospy.get_param("ardrone/front/params/Cy","180")

    def mocap_cmd_vel_cb(self,msg):
        self.mocap_vel_msg = msg
        self.mocap_vel_msg_time = rospy.get_time()

        # print(self.mocap_cmd_vel_topic + '/linear : {} {} {}' ).format(self.mocap_vel_msg.linear.x, self.mocap_vel_msg.linear.y, self.mocap_vel_msg.linear.z)
        # print(self.mocap_cmd_vel_topic + '/angular : {} {} {}').format(self.mocap_vel_msg.angular.x, self.mocap_vel_msg.angular.y, self.mocap_vel_msg.angular.z)

    def facetracker_centroid_cb(self,msg):
        self.face_centroid_msg = msg
        self.face_centroid_msg_time = rospy.get_time()
        print(self.face_centroid_topic + ' : {} {} {}' ).format(self.face_centroid_msg.x, self.face_centroid_msg.y, self.face_centroid_msg.z)


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('faceTrackingAutopilot')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        fta = faceTrackingAutopilot()
    except rospy.ROSInterruptException: pass

    # Process the images when new ones arrive
    Kyaw = 0.01 # proportional gain for yaw feedback from image
    Kz = 0.01   # proportional gain for altitude feedback from image

    dummy = None
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        if (fta.face_centroid_msg_time == None): 
            # do nothing until a face is seen
            dummy = None
        else: # let the face tracker and time switcher choose
            # print ('rospy.get_time() - fta.face_centroid_msg_time = {}').format(rospy.get_time() - fta.face_centroid_msg_time)
            print('now = {}, fta.face_centroid_msg_time = {}').format(rospy.get_time(), fta.face_centroid_msg_time)
            time_since_last_face = rospy.get_time() - fta.face_centroid_msg_time
            print('time since last face was seen: {}').format(time_since_last_face) 
            if (time_since_last_face<2):
                # Then calculate cmd_vel based on faces
                dx = fta.param_image_Cx - fta.face_centroid_msg.x # This contributes to yawing
                dy = fta.param_image_Cy - fta.face_centroid_msg.y # this contributes to altitude

                # use mocap to keep uav in center of workspace, let camera control altitude and yaw-rate
                cmd_linear = Vector3(fta.mocap_vel_msg.linear.x, fta.mocap_vel_msg.linear.y, Kz*dy)
                cmd_angular = Vector3(0, 0, Kyaw*dx)

                fta.switched_cmd_vel_msg = Twist(cmd_linear, cmd_angular)
            elif(
                    time_since_last_face>2 and 
                    time_since_last_face<4
                ):
                # do nothing, maybe the face will come back
                fta.switched_cmd_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            else:
                # lost the face, go home
                fta.switched_cmd_vel_msg = fta.mocap_vel_msg

            #publish cmd_vel no matter what
            fta.switched_cmd_vel_pub.publish(fta.switched_cmd_vel_msg)
        rate.sleep()
