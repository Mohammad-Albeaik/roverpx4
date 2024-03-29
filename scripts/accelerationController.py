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
sum_error 		= 0


goal       = 0.4

kp         = 1	#P for linear
kd         = 0.0     #D for linear
ki         = 0.0  #I for linear


throttle     = 0
acceleration = 0
prev_vel     = 0
dpwm         = 0
T = 0
v = 0
prev_error = 0

def spCb(msg):
    global prev_vel, acceleration, Ti, T, v
    xddot    = msg.linear.x
    yddot    = msg.linear.y
    forward = msg.linear.z

    T  = time.time()
    #dT = T - Ti

    v = forward*sqrt(xddot*xddot+yddot*yddot)
    #acceleration = (v - prev_vel)/dT

    #prev_vel = v
    #Ti = T

    acceleration = sqrt(xddot*xddot + yddot*yddot)
#    print(yddot)
    #### if the acceleration is noisy, try to filter T or dT before using calculating the acceleration



def UpdateSpeed():
    global v, acceleration, sum_error, kp, kd, ki, T, T0, throttle, dpwm, prev_error

#------------------------------------------------------------------------- PID Start ----------------------------------------------------------------------

    error = goal - acceleration


    sum_error = sum_error + error

    throttle = throttle + (kp * error + kd * prev_error + ki * sum_error)

    # if acceleration does not depend on the throttle, but the change in throttle
    # maybe try to control the increment in throttle.
    # so that the throttle will be changing by dpwm... dpwm could be +- or 0
    # the PID tries to find the value of dpwm

   # dpwm = dpwm + (kp * error + kd * prev_error + ki * sum_error)
   # throttle = throttle + dpwm

    if sum_error > 10:
        sum_error = 10
    elif sum_error < -10:
        sum_error = -10


    prev_error = error






#------------------------------------------------------------------------- PID end-------------------------------------------------------------------------
    if throttle >= 0:
        pwm = - throttle + 1480
    if throttle < 0:
        pwm = - throttle + 1520

    if pwm > 1950:
        pwm = 1950
    if pwm < 1050:
        pwm = 1050

#    print(round(error,2), "---",round(acceleration,3), "pwm",round(pwm,1))
   # pwm = 1500
    RcOver.channels = [1500,pwm,1500,pwm-20,0,0,0,0]
    print(round(acceleration,5))
#    print(round(v,3)," <-- V, a --> ",round(acceleration,3))
#    print(round(v,3)," <-- V, Time --> ",round(T-T0,3))
   # print(round(acceleration,3)," <-- a, Time --> ",round(T-T0,3))






# Main function
def main():
    global T0, Ti

    time.sleep(3.5)

    # Initiate node
    rospy.init_node('Roveer_acceleration', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(40.0)

    # Subscribe to Rover's local position
    rospy.Subscriber('acceleration',Twist, spCb)


    # RCOveride publisher
    rc_pub = rospy.Publisher('mavros/rc/override',OverrideRCIn, queue_size=1)

    T0 = time.time()
    Ti = T0
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
