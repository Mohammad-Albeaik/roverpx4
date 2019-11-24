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
import scipy.io
import time




RcOver                  = OverrideRCIn()
RcOver.channels = [1500,1500,1500,1500,0,0,0,0]
M       = []
alpha   = []
beta    = []
prev_s  = []
prev_v  = []

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
def ControllerMatrices()

    allmat  = scipy.io.loadmat('data1.mat')
    M       = allmat.get('M')
    alpha   = allmat.get('alpha')
    beta    = allmat.get('beta')
    M       = np.array(M) # For converting to a NumPy array
    alpha   = np.array(alpha) # For converting to a NumPy array
    beta    = np.array(beta) # For converting to a NumPy array


def UpdateAcceleration():
   # global x, y, v, w, vdot, lx, ly, lv, lw  
  
    desired_distance = 0.25
    d = sqrt((x-lx)*(x-lx) + (y-ly)*(y-ly)) - desired_distance
    #s = 0.0476*v + 0.9524*prev_s + 0.0976*u
    s = v + vdot
    states = [lv, d, v, s]

    scaling  = np.max(np.max(np.matmul(M,x)),0.01) # Prevent division by zero
    umax     = np.min(np.matmul(alpha,x).reshape(len(beta),1)/scaling + beta)
    umin     = np.max(np.matmul(alpha,x).reshape(len(beta),1)/scaling - beta)
    u        = scaling*(umax+umin)/2
    
    input_acceleration = - v + (0.0476*v + 0.9524*s + 0.0976*u)
    prev_s = s
    prev_v = v


    #vi = vi + 13.4498* input_acceleration/42.935
    vi = vi + 0.3396*input_acceleration
    r = 0.065
    RcOver.channels = [1500, vi/r,1500, vi/r-20,0,0,0,0]   # 4th

    

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

    # ROS main loop
    while not rospy.is_shutdown():

            UpdateAcceleration()
            rc_pub.publish(RcOver)
            rate.sleep()


if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass
