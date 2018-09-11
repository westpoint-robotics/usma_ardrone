#!/usr/bin/env python

# Import required Python code.
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3
from os.path import expanduser
home = expanduser("~")
import time

class face_tracker():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # define ros publishers and subscribers
        self.bridge = CvBridge()
        self.pub_centroid = rospy.Publisher('/face_centroid', Vector3, queue_size=1)  

        # subscribe to param image topic
        self.image_topic = rospy.get_param('~subscribed_image_topic', '/ardrone/image_raw')
        self.image_sub = rospy.Subscriber("image_topic",Image,self.imgsub)
        
    def callback(self,data):
        # call back to read image message and save it into the class
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.new_image = True
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('face_tracker')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ft = face_tracker()
    except rospy.ROSInterruptException: pass

    # Load params if provided else use the defaults
    output_image_topic = rospy.get_param("output_image_topic","/face_detector/raw_image")
    haar_file_face = home + "/" + rospy.get_param("haar_file_face","/ros/src/face_shooter/data/face.xml")
    face_tracking = rospy.get_param("face_tracking","1")
    display_original_image = rospy.get_param("display_original_image","1")
    display_tracking_image = rospy.get_param("display_tracking_image","1")
    # image_width = 
    # image_height = 

    # Create the classifier
    face_cascade=cv2.CascadeClassifier(haar_file_face)

    # Process the images when new ones arrive
    last_time = 0
    num_frames = 0
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        if (ft.new_image):
            # convert new image to gray scale
            gray = cv2.cvtColor(ft.cv_image, cv2.COLOR_BGR2GRAY)
            # Use the classifier to find faces
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            center=Vector3()
            for index,(x,y,w,h) in enumerate(faces): # For each face publish the centroid
                #print("Frame is: %d by %d and x,y %d, %d" %(frame.shape[1],frame.shape[0],x,y))
                frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2) # Draw a frame around each face
                # publish center of face
                center.x=x+w/2
                center.y=y+h/2
                center.z=index+1 # unique id for multiple faces, zero indexed
            if center.z != 0: # then at least one face was found
                pub.publish(center)
        # wait for new image to arrive
        rate.sleep() 
    # when done
    cv2.destroyAllWindows()
