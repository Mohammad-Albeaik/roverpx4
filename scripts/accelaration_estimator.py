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


x         = 0.0
y         = 0.0
z         = 0.0
local_ang = [0.0, 0.0, 0.0, 0.0]
roll      = 0.0
pitch     = 0.0
yaw       = 0.0
FirstTime = 1
velocity = Twist()

xold 		= 0
yold 		= 0
secondsold = 0
micro_secold = 0
b 			= [0.00002915    ,0.00008744,    0.00008744   , 0.00002915]
a 			= [1.0000  , -2.8744   , 2.7565  , -0.8819]
by                          = [0.0009, 0.0019, 0.0009]
ay                          = [1,-1.9112, 0.915]
x_dot_memory 	        	= [0.00,    0.00,    0.00,  0.00]
filtered_x_dot_memory 		= [0.00,    0.00,    0.00,  0.00]
y_dot_memory 		        = [0.00,    0.00,    0.00,  0.00]
filtered_y_dot_memory 		= [0.00,    0.00,    0.00,  0.00]
yspeedold = 0
xspeedold = 0
converion = 1000000


def posCb(msg):
    global x, y, z, roll, pitch, yaw
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    local_ang = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(local_ang)




def UpdateVelocity():
    global velocity, yaw, x, y, FirstTime, xold, yold,xspeedold, yspeedold,secondsold,micro_secold, a,b, x_dot_memory, y_dot_memory, filtered_x_dot_memory, filtered_y_dot_memory,ay,by,converion

    now = rospy.get_rostime()
    seconds 	= now.secs
    micro_sec 	= now.nsecs/1000
    dT  	 	= (seconds - secondsold)*1000000 + (micro_sec - micro_secold)      # time in seconds



    xspeed 		= ((x - xold)/(dT))*converion 			# m/s
    yspeed 		= ((y - yold)/(dT))*converion      # m/s


    xacceleration               = ((xspeed - xspeedold)/(dT))*converion                   # m/s
    yacceleration               = ((yspeed - yspeedold)/(dT))*converion      # m/s


#    print(yacceleration)
#------------------------------------------------------------------------- filter -------------------------------------------------------------------------
    x_dot_memory[0] = x_dot_memory[1];
    x_dot_memory[1] = x_dot_memory[2];
    x_dot_memory[2] = x_dot_memory[3];
    x_dot_memory[3] = xacceleration;
    filtered_x_dot_memory[0] = filtered_x_dot_memory[1];
    filtered_x_dot_memory[1] = filtered_x_dot_memory[2];
    filtered_x_dot_memory[2] = filtered_x_dot_memory[3];
    filtered_x_dot_memory[3] = (float) ( (b[0]*x_dot_memory[3] + b[1]*x_dot_memory[2] + b[2]*x_dot_memory[1]+ b[3]*x_dot_memory[0] - a[1]*filtered_x_dot_memory[2] - a[2]*filtered_x_dot_memory[1]- a[3]*filtered_x_dot_memory[0]));

    y_dot_memory[0] = y_dot_memory[1];
    y_dot_memory[1] = y_dot_memory[2];
    y_dot_memory[2] = y_dot_memory[3];
    y_dot_memory[3] = yacceleration;
    filtered_y_dot_memory[0] = filtered_y_dot_memory[1];
    filtered_y_dot_memory[1] = filtered_y_dot_memory[2];
    filtered_y_dot_memory[2] = filtered_y_dot_memory[3];
    filtered_y_dot_memory[3] = (float) ( (b[0]*y_dot_memory[3] + b[1]*y_dot_memory[2] + b[2]*y_dot_memory[1] + b[3]*y_dot_memory[0] - a[1]*filtered_y_dot_memory[2] - a[2]*filtered_y_dot_memory[1] - a[3]*filtered_y_dot_memory[0] ));

#-------------------------------------------------------------------------filter end-------------------------------------------------------------------------
    xfdot = cos(yaw)*velocity.linear.x + sin(yaw)*velocity.linear.y
    if xfdot >0:
       velocity.linear.z = 1
    else:
       velocity.linear.z = -1

    velocity.linear.x = round(filtered_x_dot_memory[2],4)
    velocity.linear.y = round(filtered_y_dot_memory[2],4)


    xold 		= x
    yold 		= y
    xspeedold = xspeed
    yspeedold = yspeed
    secondsold	= seconds
    micro_secold = micro_sec
    print(velocity.linear.y)
#    print("speed ",sqrt(velocity.linear.x *velocity.linear.x + velocity.linear.y*velocity.linear.y))
   # print(velocity)

# Main function
def main():

    # Initiate node
    rospy.init_node('acceleration_estimator', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(90.0) 

    # Subscribe to drone's local position
    rospy.Subscriber('vrpn_client_node/mmb/pose', PoseStamped, posCb)

    # Speed publisher
    velocity_pub = rospy.Publisher('acceleration',Twist, queue_size=1)


    # ROS main loop
    while not rospy.is_shutdown():

            UpdateVelocity()
            velocity_pub.publish(velocity)
            rate.sleep()


if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass