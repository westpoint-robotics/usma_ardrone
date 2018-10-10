#!/usr/bin/env python

# Import required Python code.
import math
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Vector3, Twist #TwistStamped
import time
import tf

class vrpn_heading():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # check to see if data should be logged:
        #subscribe to optitrack cmd_vel 
        self.mocap_cmd_vel_topic = "/vrpn_client_node/Ardrone/pose"
        self.mocap_vel_sub = rospy.Subscriber(self.mocap_cmd_vel_topic,PoseStamped,self.mocap_pose_cb)

    def mocap_pose_cb(self,msg):
        orientation_q = msg.pose.orientation
        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
        yaw = self.yaw_from_q(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        # print(('vrpn_heading: {}').format(msg.pose.orientation.w))
        print(('vrpn_heading: {}').format(yaw))

    def yaw_from_q(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))
        return X


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('vrpn_heading')
    try:
        vrpnh = vrpn_heading()
    except rospy.ROSInterruptException: pass

    dummy = None
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():

        rate.sleep()

# end main