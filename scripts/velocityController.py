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
from mavros_msgs.msg import OverrideRCIn


xdot         			= 0.0
ydot         			= 0.0
yawdot       			= 0.0

RcOver 					= OverrideRCIn()
RcOver.channels = [1500,1500,1500,1500,0,0,0,0]
sum_error_v 		= 0
sum_error_w 		= 0

kvp 							= 20				## Speed PID controller
kvd                             = 2                 # Speed PID controller
kvi                             = 0.05                 # Speed PID controller
kwp 							= 50				# Speed PID controller
kwd 							= 2					# Speed PID controller
kwi 							= 0.05 				# Speed PID controller

old_v 					    = 0
old_w 					    = 0

vg 						= 0
wg 						= 0




def spCb(msg):
    global xdot, ydot, yawdot,v,w
    xdot = msg.linear.x
    ydot = msg.linear.y
    yawdot = msg.angular.z

    w = yawdot
    v = sqrt(xdot*xdot+ydot*ydot)



def UpdateSpeed():
    global v, w , sum_error_v, sum_error_w, kvp, kwp, kvd, kwd, kvi, kwi, old_v, old_w, vg, wg

#------------------------------------------------------------------------- PID Start ----------------------------------------------------------------------
    vg = 0
    wg = 0

    dv = vg - v
    dw = wg - w


    sum_error_v = sum_error_v + v
    sum_error_w = sum_error_w + w

    vi =  kvp * v   + kvd * (abs(v) - abs(old_v)) + kvi * sum_error_v
    wi =  kwp * w + kwd * (abs(w) - abs(old_w)) + kwi * sum_error_w

    if sum_error_v > 10:
        sum_error_v = 10
    elif sum_error_v < -10:
        sum_error_v = -10
    if sum_error_w > 10:
        sum_error_w = 10
    elif sum_error_w < -10:
        sum_error_w = -10

    if vi > 30:
        vi = 30
    if vi < -30:
        vi = -30
    if wi > 90:
        wi = 90
    if wi < -90:
        wi = -90

    old_v       = v
    old_w       = w


 


    
#------------------------------------------------------------------------- PID end-------------------------------------------------------------------------
    r = 0.13 # radius 13 cm
    L = 0.33 # length between wheels 33 cm


    WR =   vi/r +L/r * wi
    WL =   2* vi/r - WR

    if WR > 0:
        WR = - WR + 1405
    elif WR <0:
        WR = - WR +1595
    else:
        WR = 1500
    if WL > 0:
        WL = - WL + 1405
    elif WL < 0:
        WL = - WL +1595
    else:
        WL = 1500


    RcOver.channels = [1500,WR,1500,WL,0,0,0,0]   # 4th





# Main function
def main():

    # Initiate node
    rospy.init_node('Roveer_sp', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(60.0)

    # Subscribe to Rover's local position
    rospy.Subscriber('velocity',Twist, spCb)


    # RCOveride publisher
    rc_pub = rospy.Publisher('mavros/rc/override',OverrideRCIn, queue_size=1)


    # ROS main loop
    while not rospy.is_shutdown():

            UpdateSpeed()
            rc_pub.publish(RcOver)
            rate.sleep()


if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass
