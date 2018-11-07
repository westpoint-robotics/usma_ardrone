#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion



class talker():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.pub = rospy.Publisher('/usma_ardrone/mpc/desiredpose', PoseStamped, queue_size=10)
        rospy.init_node('desiredpose_publisher', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        self.pub_msg = PoseStamped()
        self.pub_msg.pose.position = Vector3(0.75, -0.5, 1.350)
        self.pub_msg.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)


        yaw = 90*3.14159/180
        self.qz = math.sin(yaw/2)
        self.qw = math.cos(yaw/2)

        while not rospy.is_shutdown():
            self.set_waypoint()
            self.pub_msg.header.stamp = rospy.get_rostime()
            self.pub.publish(self.pub_msg)
            rate.sleep()

    def set_waypoint(self):
        msg_time = rospy.get_time()
        modulo = msg_time%40
        if modulo < 10.0:
            self.pub_msg.pose.position = Vector3(1.0, 1.0, 1.350)
            self.pub_msg.pose.orientation = Quaternion(0.0, 0.0, self.qz, self.qw)
        elif 10.0 < modulo <= 20.0:
            self.pub_msg.pose.position = Vector3(1.0, -1.0, 1.350)
            self.pub_msg.pose.orientation = Quaternion(0.0, 0.0, self.qz, self.qw)
        elif 20.0 < modulo <= 30.0:
            self.pub_msg.pose.position = Vector3(-1.0, -1.0, 1.350)
            self.pub_msg.pose.orientation = Quaternion(0.0, 0.0, self.qz, self.qw)
        else:
            self.pub_msg.pose.position = Vector3(-1.0, 1.0, 1.350)
            self.pub_msg.pose.orientation = Quaternion(0.0, 0.0, self.qz, self.qw)


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