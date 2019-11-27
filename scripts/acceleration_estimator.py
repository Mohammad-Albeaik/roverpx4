#!/usr/bin/env python
# ROS python API
import rospy
# Joy message structure
# 3D point & Stamped Pose msgs & Orientation as quaternion
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
#import math for arctan and sqrt function
from math import atan2, sqrt, pi, cos, sin
#import quaternion transformation
from tf.transformations import euler_from_quaternion
from numpy import array
from std_msgs.msg import String


acceleration = Twist()

xdot_old        = 0
ydot_old        = 0

secondsold      = 0
micro_secold    = 0
b                           = [0.0075,    0.0151,    0.0075]
a                           = [1.0000,   -1.7399,    0.7700]

x_ddot_memory                = [0.00,    0.00,    0.00]
filtered_x_ddot_memory       = [0.00,    0.00,    0.00]
y_ddot_memory                = [0.00,    0.00,    0.00]
filtered_y_ddot_memory       = [0.00,    0.00,    0.00]

xdot    = 0
ydot    = 0
v       = 0
prev_v  = 0

def spCb(msg):
    global v,w, xdot, ydot
    xdot    = msg.linear.x
    ydot    = msg.linear.y
    forward = msg.linear.z
    v = forward*sqrt(xdot*xdot+ydot*ydot)



def UpdateVelocity():
    global acceleration,v,prev_v, xdot_old, ydot_old,secondsold,micro_secold, a,b, x_ddot_memory, y_ddot_memory, filtered_x_ddot_memory, filtered_y_ddot_memory, xdot,ydot

    now = rospy.get_rostime()
    seconds     = now.secs
    micro_sec   = now.nsecs/1000
    dT          = (seconds - secondsold)*1000000 + (micro_sec - micro_secold)      # time in seconds



    xacceleration      = ((xdot - xdot_old)/(dT))*1000000             # m/s
    yacceleration      = ((ydot - ydot_old)/dT)*1000000      # m/s

 #   print(yacceleration)

#------------------------------------------------------------------------- filter -------------------------------------------------------------------------
    x_ddot_memory[0] = x_ddot_memory[1];
    x_ddot_memory[1] = x_ddot_memory[2];
    x_ddot_memory[2] = xacceleration;
    filtered_x_ddot_memory[0] = filtered_x_ddot_memory[1];
    filtered_x_ddot_memory[1] = filtered_x_ddot_memory[2];
    filtered_x_ddot_memory[2] = (float) ( (b[0]*x_ddot_memory[2] + b[1]*x_ddot_memory[1] + b[2]*x_ddot_memory[0] - a[1]*filtered_x_ddot_memory[1] - a[2]*filtered_x_ddot_memory[0]));

    y_ddot_memory[0] = y_ddot_memory[1];
    y_ddot_memory[1] = y_ddot_memory[2];
    y_ddot_memory[2] = yacceleration;
    filtered_y_ddot_memory[0] = filtered_y_ddot_memory[1];
    filtered_y_ddot_memory[1] = filtered_y_ddot_memory[2];
    filtered_y_ddot_memory[2] = (float) ( (b[0]*y_ddot_memory[2] + b[1]*y_ddot_memory[1] + b[2]*y_ddot_memory[0] - a[1]*filtered_y_ddot_memory[1] - a[2]*filtered_y_ddot_memory[0]));

   #-------------------------------------------------------------------------filter end-------------------------------------------------------------------------

    if (v - prev_v)>0:
        acceleration.linear.z = 1
    else:
        acceleration.linear.z = -1

    acceleration.linear.x = round(filtered_x_ddot_memory[2],4)
    acceleration.linear.y = round(filtered_y_ddot_memory[2],4)

    xdot_old        = xdot
    ydot_old        = ydot
    prev_v          = v
    secondsold      = seconds
    micro_secold    = micro_sec

#    print(acceleration.linear.z*sqrt(acceleration.linear.x *acceleration.linear.x + acceleration.linear.y*acceleration.linear.y))

# Main function
def main():

    # Initiate node
    rospy.init_node('acceleration_estimator', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(60.0) 

    # Subscribe to Rover's local position
    rospy.Subscriber('velocity',Twist, spCb)

    # Speed publisher
    acceleration_pub = rospy.Publisher('acceleration',Twist, queue_size=1)


    # ROS main loop
    while not rospy.is_shutdown():

            UpdateVelocity()
            acceleration_pub.publish(acceleration)
            rate.sleep()


if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass
