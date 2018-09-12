
#!/usr/bin/env python
import roslib; roslib.load_manifest('amosero_bringup')
import rospy
from math import cos
from math import sin
from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from datetime import datetime
from tf.broadcaster import TransformBroadcaster

import tf

PI = 3.141592
grad2rad = PI/180.0

ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]

ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]

ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


br = tf.TransformBroadcaster()  

wheel_radius_cm = 3.5
wheel_diameter = 2 * 3.1416 * wheel_radius_cm;
wheel_max_rpm = 170
wheel_max_speed = 255

wheel_max_cm_meters_per_minute = wheel_diameter * wheel_max_rpm
wheel_max_meters_per_minute = wheel_max_cm_meters_per_minute / 100
wheel_max_meters_per_second = wheel_max_meters_per_minute / 60 # 0.6

#we need to transfer rad/s to motor speeds...
#a full circle consists of 2 PI rad
# http://en.wikipedia.org/wiki/Radian 
# we rotate 360 degree in about 3.1, -> 2 Pi rad = 3.1 -> 2Pi / 3.1
maximum_rad_per_seconds = 2 * PI / 3.1;





def myround(x):
    return int(round(x) - .5) + (x > 0)


cur_linear_x = 0.0
cur_linear_y = 0.0
cur_linear_z = 0.0

cur_th = 0.0
cur_angular_z = 0.0
last_time = 0
cur_time = 0

ori_x = 0.0;
ori_y = 0.0;
ori_z = 0.0;
ori_w = 0.0;

then = 0
now = 0

#imu mesages for further processing
def imuCb(msg):
    global ori_x,ori_y,ori_z,ori_w,ori_covariance
    ori_x = msg.orientation.x
    ori_y = msg.orientation.y
    ori_z = msg.orientation.z
    ori_w = msg.orientation.w
    ori_covariance = [999999,0,0,0,9999999,0,0,0,999999]

north = 0.0;

#getting north
def northCb(msg):
    global north
    north=msg.data

# recieving command vel_messages here
def cmd_velCb(msg):

    global cur_linear_x, cur_linear_y, cur_th,cur_angular_z,last_time,cur_time,cur_linear_z,odom_pub
    global left_motor_pub,left_motor_speed_pub,right_motor_pub,right_motor_speed_pub
    global rospy,maximum_rad_per_seconds, now, then

    #check if speed is valid and in case limit it down
    calc_speed =  msg.linear.x
    if(msg.linear.x>wheel_max_meters_per_second):
        calc_speed = msg.linear.x
        if(msg.linear.x>0):
            msg.linear.x = wheel_max_meters_per_second
        else:
            msg.linear.x = - 1 * wheel_max_meters_per_second
        rospy.logwarn("Limited speed due to physical restrictions! max speed is: " + str(wheel_max_meters_per_second) + " m/s")

    # normalise speed for motor speed
    # motors take  255 < x > 511 forward
    #                0 < x < 255 backward 

    normalised_speed = calc_speed / wheel_max_meters_per_second 
    total_speed = myround(normalised_speed*255)

    if(total_speed>255): #limit maximum forward speed
        total_speed = 255
    if(total_speed<-255): #limit maximum backward speed
        total_speed = -255


    # rotation
    turning_delimiter = 0

    if(msg.angular.z != 0):        
        #if(msg.angular.z > maximum_rad_per_seconds):
        #    msg.angular.z = maximum_rad_per_seconds
        #    rospy.logwarn("Limited rotation speed due to physical restrictions! max rot speed is: " + str(maximum_rad_per_seconds) + " rad/s")
        normalised_rotating_speed = msg.angular.z / maximum_rad_per_seconds 
        if(msg.angular.z > 0): 
            total_rot_speed = myround(normalised_rotating_speed*255)
            msg.angular.z = maximum_rad_per_seconds
            #turning_delimiter = 255 - total_rot_speed
        else:
            total_rot_speed = myround(normalised_rotating_speed*255)
            msg.angular.z = -1 * maximum_rad_per_seconds
            #turning_delimiter = 255 - total_rot_speed

    #it is nice to now where north is..  
    rospy.loginfo("north: " + str(north));    

    if(msg.linear.x>0):
        if(msg.angular.z >= 0):
            left_motor_pub.publish(255+abs(total_speed))
            rospy.loginfo("Left Motor: " + str(255+abs(total_speed)))
    elif(msg.linear.x<0):
        if(msg.angular.z >= 0):
            left_motor_pub.publish(abs(total_speed))
            rospy.loginfo("Left Motor: " + str(abs(total_speed)))
    elif(msg.linear.x==0):
        if(msg.angular.z > 0):
            left_motor_pub.publish(255-turning_delimiter)
            #left_motor_pub.publish(abs(total_speed))
        elif(msg.angular.z < 0):
            left_motor_pub.publish(511-turning_delimiter)
            #left_motor_pub.publish(255+abs(total_speed))
        else:
            left_motor_pub.publish(0)
            #left_motor_speed_pub.publish(0)

    if(msg.linear.x>0):
        if(msg.angular.z <= 0):
            right_motor_pub.publish(255+abs(total_speed))
            rospy.loginfo("Right Motor: " + str(255+abs(total_speed)))
    elif(msg.linear.x<0):
        if(msg.angular.z <= 0):
            right_motor_pub.publish(abs(total_speed))
            rospy.loginfo("Right Motor: " + str(abs(total_speed)))
    elif(msg.linear.x==0):
        if(msg.angular.z < 0):
            right_motor_pub.publish(255-turning_delimiter)
            #right_motor_pub.publish(abs(total_speed))
        elif(msg.angular.z > 0):
            right_motor_pub.publish(511-turning_delimiter)
            #right_motor_pub.publish(255+abs(total_speed))
        else:
            right_motor_pub.publish(0)

    #calculate speed according to last message recieved
    
    if(last_time == 0):
        last_time = rospy.get_time() # rospy.Time.now(); rospy.get_time()
    
    cur_time = rospy.get_time() # rospy.Time.now()

    #if(then == 0):
    #    then = datetime.now()
    #now = datetime.now()
    #elapsed = now - then
    #elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.
    
    rospy.loginfo("cur_time: " +  str(cur_time) + " last_time:" +  str(last_time))

    dt = cur_time - last_time 

    #+ (cur_time.nsecs - last_time.nsecs) /1000000000;
    #dt = float(dt.seconds) + dt.microseconds/1000000.

    #dt = cur_time  - last_time;
    vx = msg.linear.x
    vy = 0
    vth = msg.angular.z

    # v = s / t -> t = s / v  -> s = v * t 

    #delta_x = (vx * cos(vth) - vy * sin(vth)) * dt
    #delta_y = (vx * sin(vth) + vy * cos(vth)) * dt

    #cur_linear_x += delta_x;
    #cur_linear_y += delta_y;

    delta_th = vth * dt
    cur_th += delta_th

    x = cos(vth)*vx / 100
    y = -sin(vth)*vx / 100

    cur_linear_x = cur_linear_x + (cos(cur_th)*x - sin(cur_th)*y)
    cur_linear_y = cur_linear_y + (sin(cur_th)*x + cos(cur_th)*y)

    rospy.loginfo("speeds: [%f, %f] time: [%f]"%(vx,vth,dt))
    rospy.loginfo("cur: [%f, %f, %f] + delta_th, cur_th [%f, %f, %f]"%(cur_linear_x, cur_linear_y, cur_th,delta_th,cur_th,dt))


    odom_quat = tf.transformations.quaternion_from_euler(0, 0, cur_th)
    #odom_quat = tf.transformations.quaternion_from_euler(0, 0, north)
    odom_quat = Quaternion(*odom_quat)



    odom_trans = geometry_msgs.msg.TransformStamped() 
    odom_trans.header.stamp = rospy.Time.now()
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint"
    odom_trans.transform.translation.x = cur_linear_x;
    odom_trans.transform.translation.y = cur_linear_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
        
    t = tf.Transformer(True, rospy.Duration(10.0))
    t.setTransform(odom_trans)

    #br.sendTransform(odom_trans);

    quaternion = Quaternion()
    quaternion.x = 0.0 
    quaternion.y = 0.0
    quaternion.z = sin(cur_th/2)
    quaternion.w = cos(cur_th/2)

    # odomBroadcaster = TransformBroadcaster() 
    # odomBroadcaster.sendTransform(
    #             (cur_linear_x, cur_linear_y, 0), 
    #             (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
    #             rospy.Time.now(),
    #             "base_footprint",
    #             "odom"
    #             )


    #TransformStamped
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.pose.pose.position.x = cur_linear_x # delta_x;
    odom.pose.pose.position.y = cur_linear_y # delta_y;
    #odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    #if(ori_x != 0):
        #odom.pose.pose.orientation.x = ori_x 
        #odom.pose.pose.orientation.y = ori_y 
        #odom.pose.pose.orientation.z = ori_z
        #odom.pose.pose.orientation.x = ori_w
        #odom.pose.pose.orientation.covariance = ori_covariance;
        
        #odom.pose.pose.orientation = odom_quat
    #else:
        
    odom.pose.pose.orientation = odom_quat;


    #odom.pose.pose.orientation.x = odom_quat.x 
    #odom.pose.pose.orientation.y = odom_quat.y
    #odom.pose.pose.orientation.z = odom_quat.z
    #odom.pose.pose.orientation.x = odom_quat.w
    
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0
    odom.twist.twist.angular.z = delta_th
    #odom.twist.twist.linear.y = ori_w;

    #if(north!=0.0):
        #odom.twist.twist.angular.z = north;
    #else:
    #    odom.twist.twist.angular.z = vth;

    #odom.twist.twist.angular.z = delta_th;


    odom.pose.covariance = ODOM_POSE_COVARIANCE
    odom.twist.covariance = ODOM_TWIST_COVARIANCE

    odom_pub.publish(odom);
    last_time = cur_time



    
def listener():
    global odom_pub,left_motor_pub,left_motor_speed_pub,right_motor_pub,right_motor_speed_pub

    #subscribing to required topics
    rospy.Subscriber("/cmd_vel", Twist, cmd_velCb)
    rospy.Subscriber('imu_data',Imu,imuCb)
    rospy.Subscriber('north',std_msgs.msg.Float32,northCb)
    
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    left_motor_pub = rospy.Publisher('/left_motor', std_msgs.msg.Int32, queue_size=10)
    right_motor_pub = rospy.Publisher('/right_motor', std_msgs.msg.Int32, queue_size=10)

    #left_motor_speed_pub = rospy.Publisher('/left_motor_speed', std_msgs.msg.Int32, queue_size=10)
    #right_motor_speed_pub = rospy.Publisher('/right_motor_speed', std_msgs.msg.Int32, queue_size=10)


    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('aMoSeRo_MotorcontrolTFBroadcast')    
        last_time =  rospy.get_time() # last_time.Time.now();
        cur_time = rospy.get_time() # rospy.Time.now(); 
        rospy.Rate(10) # 10hz

        rospy.loginfo("Using following parameters: ")
        rospy.loginfo("wheel_max_cm_meters_per_minute =  " + str(wheel_max_cm_meters_per_minute))
        rospy.loginfo("wheel_max_meters_per_minute =  " + str(wheel_max_meters_per_minute))
        rospy.loginfo("wheel_max_meters_per_second =  " + str(wheel_max_meters_per_second))

        rospy.logwarn("We will start as soon as cmd_vels are published" + str(wheel_max_meters_per_second))

        listener()                      
    except rospy.ROSInterruptException: pass