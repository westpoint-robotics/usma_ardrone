#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from usma_plugins.msg import ardrone_pose

def talker():
    pose_topic = rospy.get_param("~pose_topic",'/usma_ardrone/uav/desired_pose') # false by default, in case operator doesn't know to create data folder for logging
    pub = rospy.Publisher(pose_topic, ardrone_pose, queue_size=10)
    rospy.init_node('desiredpose_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub_msg = ardrone_pose()
    pub_msg.position = Vector3(0.0, 0.0, 1.6)
    pub_msg.heading = 0

    while not rospy.is_shutdown():
        pub.publish(pub_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


# {
#     header: 
#     {
#         seq: 1,
#         stamp: {secs: 1, nsecs: 0},
#         frame_id: ''
#     },
#     pose: 
#     {
#         position: {x: 1.0, y: 0.0, z: 1.0}, 
#         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
#     }

# }