#!/usr/bin/env python

import rospy
import cv2
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
from os.path import expanduser
home = expanduser("~")
import time

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    rospy.init_node("python_cam")
    pub = rospy.Publisher('/face_centroid', Vector3, queue_size=1)  
    
    # Load params if provided else use the defaults
    output_image_topic = rospy.get_param("output_image_topic","/face_detector/raw_image")
    haar_file_face = home + "/" + rospy.get_param("haar_file_face","catkin_ws/src/face_shooter/data/face.xml")
    face_tracking = rospy.get_param("face_tracking","1")
    display_original_image = rospy.get_param("display_original_image","1")
    display_tracking_image = rospy.get_param("display_tracking_image","1")
    
    # Create the classifier
    face_cascade=cv2.CascadeClassifier(haar_file_face)

    rate = rospy.Rate(30) # 30hz
    # Process the images

    # Start time
    last_time = time.time()
    num_frames = 0
    while not rospy.is_shutdown():
        curr_time = time.time()
        # Capture frame-by-frame
        ret, frame = cap.read() 
        num_frames += 1
        if (display_original_image): # Dispaly the image if param is set to true       
            cv2.imshow('original',frame)              
        # Gray scale image            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
            pub.publish(center)
        if center.z == 0: # No faces found so aim at center
            center.x = 320 # This is the set Point TODO use a param
            pub.publish(center)
        if (display_tracking_image): # Dispaly the image if param is set to true
            cv2.imshow('tracking',frame)     
        #except Exception as e:
        #    print(e)
        elapsed_time = curr_time - last_time
        if (elapsed_time > 1.0):
            #print("FPS: %f math is %d / %f" % ((num_frames / elapsed_time),num_frames,elapsed_time))
            last_time = curr_time
            num_frames = 0
        cv2.waitKey(1) # Needed 
        rate.sleep()
    
    cv2.destroyAllWindows()
