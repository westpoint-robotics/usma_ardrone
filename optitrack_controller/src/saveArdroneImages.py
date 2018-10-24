#!/usr/bin/env python
# ========================================= #
# Original author : Dom Larkin
# converted from face_tracker2_node.py by Benjamin Abruzzo 2018-09-12
# Used by usma_ardrone package for face tracking by an AR drone
# ========================================= #

# Import required Python code.
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3
import time

class saveArdroneImages():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.new_image = False
        self.img_time = rospy.get_time()
        # print("%% init time: {} \n\n").format(self.img_time)
        self.rate = rospy.Rate(30) # 30hz sleep rate for ros
        self.bridge = CvBridge()

        self.sequence = rospy.get_param('~sequence', 'seq_01')

        # define ros publishers and subscribers
        self.image_topic = rospy.get_param('~subscribed_image_topic', '/ardrone/image_raw')
        # print('subscribing to: ' + self.image_topic)
        self.image_sub = rospy.Subscriber(self.image_topic,Image,self.image_callback)
        self.image_seq = 0
        
    def image_callback(self,data):
        # print('image_callback(self,data):')
        # call back to read image message and save it into the class
        try:
            img_msg_time = data.header.stamp.to_sec()
            if (img_msg_time != self.img_time): # Dispaly the image if param is set to true       
                self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                self.new_image = True
                self.img_time = img_msg_time
                self.image_seq +=1

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('saveArdroneImages')
    try:
        ft = saveArdroneImages()
    except rospy.ROSInterruptException: pass

    # Process the images when new ones arrive
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        # print('while not rospy.is_shutdown():')
        if (ft.new_image):
            image_seq_filename = ('/home/benjamin/ros/data/sequences/{0}/image_{1:0>3}.png').format(ft.sequence, ft.image_seq)
            print(image_seq_filename)
            # image_seq_filename = ('/home/benjamin/ros/data/sequences/{1}}/image_{0:0>3}.m').format(ft.image_seq, ft.sequence)
            cv2.imwrite(image_seq_filename,ft.cv_image)
            # cv2.imshow('ardrone/front/rect', ft.cv_image)
            ft.new_image = False

        cv2.waitKey(1) # Needed for showing image
        # wait for new image to arrive
        rate.sleep() 
    # when done
    cv2.destroyAllWindows()
