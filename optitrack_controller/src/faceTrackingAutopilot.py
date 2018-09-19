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
            self.exp_run = rospy.get_param("~run","002")
            # print(self.exp_run)
            self.exp_date = rospy.get_param("~date","20180917")
            # print(self.exp_date)
            self.data_logger_filename = ('/home/benjamin/ros/data/{0:0>3}/{1:0>3}/fta_{1:0>3}.m').format(self.exp_date, self.exp_run)
            # print("faceTrackingAutopilot :: logger filename {} \n\n").format(self.data_logger_filename)
            self.data_logger = open(self.data_logger_filename, 'w')
            self.data_logger.write(("%%filename: {} \n\n").format(self.data_logger_filename))

        #subscribe to optitrack cmd_vel 
        self.mocap_cmd_vel_topic = rospy.get_param("~mocap_cmd_vel_topic","/usma_ardrone/cmd_vel")
        self.mocap_vel_sub = rospy.Subscriber(self.mocap_cmd_vel_topic,Twist,self.mocap_cmd_vel_cb)
        self.mocap_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.mocap_vel_msg_time = None
        self.mocap_vel_counter = 0

        #subscribe to face tracker centroid
        self.face_cmd_vel_topic = rospy.get_param("~face_cmd_vel_topic","/ardrone/face/cmd_vel")
        self.face_cmd_sub = rospy.Subscriber(self.face_cmd_vel_topic,Twist,self.facetracker_cmd_cb)
        self.face_cmd_msg = Vector3(0, 0, 0)
        self.face_cmd_msg_time = None
        self.face_cmd_counter = 0
        
        # publish either the mocap cmd_vel or the face centroid based cmd_vel
        self.switched_cmd_vel_topic = rospy.get_param("~switched_cmd_vel_topic","/ardrone/switched/cmd_vel")
        self.switched_cmd_vel_pub = rospy.Publisher(self.switched_cmd_vel_topic,Twist, queue_size=1)  
        self.switched_cmd_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.switched_cmd_vel_counter = 0

    def publish_switched_cmd_vel_msg(self):
        if(self.logging):
            self.switched_cmd_vel_counter += 1
            self.data_logger.write(("fta.face.switched_cmd_vel_msg.time({},1) = {:06.8f};\n").format(self.switched_cmd_vel_counter, rospy.get_time()))
            self.data_logger.write(("fta.face.switched_cmd_vel_msg.linear({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.switched_cmd_vel_counter, self.switched_cmd_vel_msg.linear.x, self.switched_cmd_vel_msg.linear.y, fta.switched_cmd_vel_msg.linear.z))
            self.data_logger.write(("fta.face.switched_cmd_vel_msg.angular({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.switched_cmd_vel_counter, self.switched_cmd_vel_msg.angular.x, self.switched_cmd_vel_msg.angular.y, fta.switched_cmd_vel_msg.angular.z))
        self.switched_cmd_vel_pub.publish(self.switched_cmd_vel_msg)

    def mocap_cmd_vel_cb(self,msg):
        self.mocap_vel_msg = msg
        # print(self.mocap_cmd_vel_topic + '/linear : {} {} {}' ).format(self.mocap_vel_msg.linear.x, self.mocap_vel_msg.linear.y, self.mocap_vel_msg.linear.z)
        # print(self.mocap_cmd_vel_topic + '/angular : {} {} {}').format(self.mocap_vel_msg.angular.x, self.mocap_vel_msg.angular.y, self.mocap_vel_msg.angular.z)
        self.mocap_vel_msg_time = rospy.get_time()
        if (self.logging):
            self.mocap_vel_counter += 1
            self.data_logger.write(("fta.mocap.vel_msg.time({},1) = {:06.8f};\n").format(self.mocap_vel_counter, self.mocap_vel_msg_time))
            self.data_logger.write(("fta.mocap.vel_msg.linear({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.mocap_vel_counter, self.mocap_vel_msg.linear.x, self.mocap_vel_msg.linear.y, self.mocap_vel_msg.linear.z))
            self.data_logger.write(("fta.mocap.vel_msg.angular({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.mocap_vel_counter, self.mocap_vel_msg.angular.x, self.mocap_vel_msg.angular.y, self.mocap_vel_msg.angular.z))

    def facetracker_cmd_cb(self,msg):
        self.face_cmd_msg = msg
        # print(self.face_cmd_vel_topic + ' : {} {} {}' ).format(self.face_cmd_msg.x, self.face_cmd_msg.y, self.face_cmd_msg.z)
        self.face_cmd_msg_time = rospy.get_time()
        if (self.logging):
            self.face_cmd_counter += 1
            self.data_logger.write(("fta.face.cmd_msg.time({},1) = {};\n").format(self.face_cmd_counter, self.face_cmd_msg_time))
            self.data_logger.write(("fta.face.cmd_msg.linear({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.face_cmd_counter, self.face_cmd_msg.linear.x, self.face_cmd_msg.linear.y, self.face_cmd_msg.linear.z))
            self.data_logger.write(("fta.face.cmd_msg.angular({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.face_cmd_counter, self.face_cmd_msg.angular.x, self.face_cmd_msg.angular.y, self.face_cmd_msg.angular.z))


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
        if (fta.face_cmd_msg_time == None): # use mocap until a face is found
            if(fta.face_cmd_msg_time == None): # but only if mocap has published a cmd vel
                fta.switched_cmd_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            else:
                fta.switched_cmd_vel_msg = fta.mocap_vel_msg
        else: # let the face tracker and time switcher choose
            time_since_last_face = rospy.get_time() - fta.face_cmd_msg_time
            # print('time_since_last_face: {}').format(time_since_last_face)
            if (time_since_last_face<=fta_face_ctrl_time):
                # Then use face control
                # use mocap to keep uav in center of workspace, let camera control altitude and yaw-rate
                cmd_linear = Vector3(fta.mocap_vel_msg.linear.x, fta.mocap_vel_msg.linear.y, fta.face_cmd_msg.linear.z)
                cmd_angular = Vector3(0, 0, fta.face_cmd_msg.angular.z)
                fta.switched_cmd_vel_msg = Twist(cmd_linear, cmd_angular)
                # print('cmd_linear  = [{:06.8f} {:06.8f} {:06.8f}]').format(fta.mocap_vel_msg.linear.x, fta.mocap_vel_msg.linear.y, fta.face_cmd_msg.linear.z)
                # print('cmd_angular = [{:06.8f} {:06.8f} {:06.8f}]').format(0, 0, fta.face_cmd_msg.angular.z)
            elif(
                    time_since_last_face>fta_face_ctrl_time and 
                    time_since_last_face<=fta_wait_for_face
                ): # do nothing, maybe the face will come back
                fta.switched_cmd_vel_msg = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            else: # lost the face, go back to mocap control
                fta.switched_cmd_vel_msg = fta.mocap_vel_msg

        #publish cmd_vel no matter what
        fta.publish_switched_cmd_vel_msg()
        rate.sleep()

    fta.data_logger.close()
# end main