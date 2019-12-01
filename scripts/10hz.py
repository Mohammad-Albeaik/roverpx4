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
import numpy as np
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
import scipy.io
import time
import os



RcOver                  = OverrideRCIn()
RcOver.channels = [1500,1500,1500,1500,0,0,0,0]
#M       = np.empty(1)
#alpha   = np.empty(1)
#beta    = np.empty(1)
prev_s  = 0
prev_v  = 0
x       = 0
y       = 0
lx       = 0
ly       = 0
vdot       = 0
wdot       = 0
v   	= 0
w 	= 0
lv 	=0
lw	=0
yawdot = 0
vi = 2
u = 0
states = [0,0,0,0]
states = np.array(states).reshape(4,1)
Ti = time.time()
input_acceleration = 0


#............................ agent's callback functions 

def posCb(msg):
    global x, y
    x = msg.pose.position.x
    y = msg.pose.position.y

def vCb(msg):
    global v,w
    xdot    = msg.linear.x
    ydot    = msg.linear.y
    forward = msg.linear.z
    w = yawdot
    v = forward*sqrt(xdot*xdot+ydot*ydot)

def acCb(msg):
    global vdot
    xddot    = msg.linear.x
    yddot    = msg.linear.y
    forward = msg.linear.z
    vdot = forward*sqrt(xddot*xddot+yddot*yddot)

#............................ leader callback functions 

def lposCb(msg):
    global lx, ly
    lx = msg.pose.position.x
    ly = msg.pose.position.y

def lvCb(msg):
    global lv,lw
    xdot    = msg.linear.x
    ydot    = msg.linear.y
    forward = msg.linear.z
    lw = yawdot
    lv = forward*sqrt(xdot*xdot+ydot*ydot)


#....................... loading controller matrices ....
def ControllerMatrices():
    global M, alpha, beta,A,B
    allmat  = scipy.io.loadmat('../catkin_ws/src/roverpx4/scripts/10hz.mat')
    M       = allmat.get('M')
    alpha   = allmat.get('alpha')
    beta    = allmat.get('beta')
    A       = allmat.get('A')
    B       = allmat.get('B2')
    M       = np.array(M) # For converting to a NumPy array
    alpha   = np.array(alpha) # For converting to a NumPy array
    beta    = np.array(beta) # For converting to a NumPy array
    A       = np.array(A) # For converting to a NumPy array
    B       = np.array(B) # For converting to a NumPy array


def controller():
    global x, y, v, w, vdot, lx, ly, lv, lw, M, alpha, beta, u, prev_v, prev_s, states, Ti, A,B,input_acceleration

    T = time.time()
    dT = T - Ti
    Ti = T

    desired_distance = 0.75
    d = sqrt((x-lx)*(x-lx) + (y-ly)*(y-ly)) - desired_distance
#    v = prev_v + vdot * dT
    s = v + vdot
    s = (v-0.9048*prev_v)/0.0952
#    print(v,states[2])
    states = [lv, d, v, s]
    states = np.array(states).reshape(4,1)

    scaling  = np.max(np.max(np.matmul(M,states)),0.01) # Prevent division by zero
    umax     = np.min(np.matmul(alpha,states).reshape(len(beta),1)/scaling + beta)
    umin     = np.max(np.matmul(alpha,states).reshape(len(beta),1)/scaling - beta)
    u        = scaling*(umax+umin)/2

    states = np.matmul(A,states.reshape(4,1))+B*u
#    print(states)
    #input_acceleration = - v + newS
    input_acceleration = (states[2]-v)/0.025
    prev_v = v




def UpdateAcceleration():
    global input_acceleration, vi, states,x,y,lx,ly

 #   vi = vi + 0.3396*input_acceleration
   # vi = vi + 0.3396*input_acceleration
    vi = 13.45* states[2] + 2.15

    r = 0.065
    if vi >0:
        pwm = 1480 - vi/r
    if vi <0:
        pwm = 1520 - vi/r
    if vi ==0:
        pwm = 1500

    if pwm < 1200:
        pwm = 1200
    if pwm > 1800:
        pwm = 1800

#    pwm = 1500
#    print(states[2])
    print(sqrt((x-lx)*(x-lx) + (y-ly)*(y-ly)) - 0.75)
    RcOver.channels = [1500, pwm,1500, pwm-20,0,0,0,0]   # 4th



# Main function
def main():

    time.sleep(3.5)

    # Initiate node
    rospy.init_node('jeffController', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(40.0)

    # Subscribe to leader's local position
    rospy.Subscriber('vrpn_client_node/leader/pose', PoseStamped, lposCb) 
    # Subscribe to leader's local velocity
    rospy.Subscriber('velocity_leader',Twist, lvCb)

    # Subscribe to your local position
    rospy.Subscriber('vrpn_client_node/mmb/pose', PoseStamped, posCb)
    # Subscribe to your local velocity
    rospy.Subscriber('velocity',Twist, vCb)
    # Subscribe to your local acceleration
    rospy.Subscriber('acceleration',Twist, acCb)


    rc_pub = rospy.Publisher('mavros/rc/override',OverrideRCIn, queue_size=1)

    ControllerMatrices()
    counter = 0

    # ROS main loop
    while not rospy.is_shutdown():
            counter = counter +1
            if counter%10 ==0:
              controller()
            UpdateAcceleration()
            rc_pub.publish(RcOver)
            rate.sleep()


if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass
