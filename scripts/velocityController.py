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
import time

RcOver 					= OverrideRCIn()
RcOver.channels = [1500,1500,1500,1500,0,0,0,0]
sum_error_v 		= 0
sum_error_w 		= 0

vg          = 2
wg          = 0
v           = 0
w           = 0

kvp         = 0.35	#P for linear
kvd         = 0.0     #D for linear
kvi         = 0.0  #I for linear
kwp         = 0.4	#P for angular
kwd         = 0.0		#D for angular
kwi         = 0.00 	#I for angular

old_v       = 0
old_w       = 0
vi = 0
wi = 0

xdot                   = 0.0
ydot                   = 0.0
yawdot                 = 0.0


def spCb(msg):
    global v,w
    xdot    = msg.linear.x
    ydot    = msg.linear.y
    forward = msg.linear.z
    yawdot  = msg.angular.z

    w = yawdot
    v =forward*sqrt(xdot*xdot+ydot*ydot)



def UpdateSpeed():
    global v, w , sum_error_v, sum_error_w, kvp, kwp, kvd, kwd, kvi, kwi, old_v, old_w, vg, wg,vi,wi,T0

#------------------------------------------------------------------------- PID Start ----------------------------------------------------------------------


    dv = vg - v
    dw = wg - w
    if dv >=1 or dv<=-1:
       kvp = 0.7
    elif dv>= 0.75 or dv<=-0.75:
       kvp = 0.45
    elif dv>= 0.45 or dv<=-0.45:
       kvp = 0.35
    else:
       kvp = 0.25

    sum_error_v = sum_error_v + dv
    sum_error_w = sum_error_w + dw

    vi = vi + (kvp * dv + kvd * old_v + kvi * sum_error_v)
    wi = wi + (kwp * dw + kwd * old_w + kwi * sum_error_w)

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

    old_v       = dv
    old_w       = dw


    vi = 13.4498* vg + 2.14981
    vi =0

#------------------------------------------------------------------------- PID end-------------------------------------------------------------------------
    r = 0.065 # radius 13 cm
    L = 0.33 # length between wheels 33 cm


    WR =   vi/r +L/r * wi
    WL =   2* vi/r - WR

    if WR > 0:
        WR = - WR + 1480
    elif WR <0:
        WR = - WR +1520
    else:
        WR = 1500
    if WL > 0:
        WL = - WL + 1480
    elif WL < 0:
        WL = - WL +1520
    else:
        WL = 1500


#    print(round(dv,3))
    print(v)
    RcOver.channels = [1500,WR,1500,WL,0,0,0,0]   # 4th





# Main function
def main():
    global T0

    time.sleep(3.5)

    # Initiate node
    rospy.init_node('Roveer_sp', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(40.0)

    # Subscribe to Rover's local position
    rospy.Subscriber('velocity',Twist, spCb)


    # RCOveride publisher
    rc_pub = rospy.Publisher('mavros/rc/override',OverrideRCIn, queue_size=1)

    T0 = time.time()
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
