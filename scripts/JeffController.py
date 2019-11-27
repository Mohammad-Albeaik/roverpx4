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
a = [   0,
      0,
    0.2061,
    0.5548,
    0.8006,
    0.9712,
    1.0867,
    1.1622,
    1.2086,
    1.2337,
    1.2434,
    1.2423,
    1.2333,
    1.2185,
    1.2000,
    1.1788,
    1.1559,
    1.1317,
    1.1068,
    1.0817,
    1.0565,
    1.0314,
    1.0064,
    0.9818,
    0.9576,
    0.9337,
    0.9102,
    0.8872,
    0.8647,
    0.8426,
    0.8209,
    0.7997,
    0.7789,
    0.7585,
    0.7385,
    0.7190,
    0.6998,
    0.6811,
    0.6626,
    0.6446,
    0.6269,
    0.6095,
    0.5925,
    0.5759,
    0.5595,
    0.5435,
    0.5278,
    0.5123,
    0.4973,
    0.4825,
    0.4680,
    0.4538,
    0.4401,
    0.4266,
    0.4134,
    0.4006,
    0.3882,
    0.3762,
    0.3646,
    0.3533,
    0.3425,
    0.3321,
    0.3220,
    0.3125,
    0.3033,
    0.2945,
    0.2862,
    0.2783,
    0.2707,
    0.2635,
    0.2568,
   -0.4816,
   -1.1896,
   -1.2716,
   -1.7107,
   -2.1682,
   -2.2962,
   -2.3073,
   -2.3013,
   -2.2825,
   -2.2544,
   -2.2197,
   -2.1805,
   -2.1382,
   -2.0940,
   -2.0487,
   -2.0028,
   -1.9569,
   -1.9111,
   -1.8658,
   -1.8211,
   -1.7772,
   -1.7340,
   -1.6918,
   -1.6504,
   -1.6099,
   -1.5702,
   -1.5314,
   -1.4935,
   -1.4565,
   -1.4204,
   -1.3850,
   -1.3504,
   -1.3167,
   -1.2838,
   -1.2516,
   -1.2201,
   -1.1893,
   -1.1593,
   -1.1300,
   -1.1012,
   -1.0731,
   -1.0457,
   -1.0188,
   -0.9926,
   -0.9669,
   -0.9417,
   -0.9171,
   -0.8931,
   -0.8695,
   -0.8465,
   -0.8238,
   -0.8017,
   -0.7800,
   -0.7588,
   -0.7380,
   -0.7177,
   -0.6977,
   -0.6782,
   -0.6590,
   -0.6404,
   -0.6221,
   -0.6042,
   -0.5868,
   -0.5698,
   -0.5531,
   -0.5369,
   -0.5211,
   -0.5059,
   -0.4911,
   -0.4768,
    0.0066,
    0.6255,
    0.8748,
    0.9624,
    0.9386,
    0.9154,
    0.8928,
    0.8708,
    0.8493,
    0.8283,
    0.8079,
    0.7924,
    0.8563,
    0.9527,
    0.9874,
    0.9901,
    0.9761,
    0.9538,
    0.9277,
    0.9001,
    0.8723,
    0.8448,
    0.8181,
    0.7923,
    0.7673,
    0.7433,
    0.7202,
    0.6980,
    0.6693,
    0.6437,
    0.6204,
    0.5988,
    0.5807,
    0.5626,
    0.5467,
    0.5309,
    0.5167,
    0.5026,
    0.4895,
    0.4766,
    0.4645,
    0.4526,
    0.4411,
    0.4300,
    0.4193,
    0.4089,
    0.3987,
    0.3889,
    0.3793,
    0.3699,
    0.3608,
    0.3519,
    0.3432,
    0.3348,
    0.3265,
    0.3185,
    0.3106,
    0.3029,
    0.2954,
    0.2881,
    0.2810,
    0.28,
    0.5,
    0.2]
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
    global M, alpha, beta
    allmat  = scipy.io.loadmat('../bigmatrices.mat')
    M       = allmat.get('M')
    alpha   = allmat.get('alpha')
    beta    = allmat.get('beta')
    M       = np.array(M) # For converting to a NumPy array
    alpha   = np.array(alpha) # For converting to a NumPy array
    beta    = np.array(beta) # For converting to a NumPy array


def UpdateAcceleration():
    global x, y, v, w, vdot, lx, ly, lv, lw, vi, M, alpha, beta, u, prev_v, prev_s, states,a

    desired_distance = 0.75
    d = sqrt((x-lx)*(x-lx) + (y-ly)*(y-ly)) - desired_distance
    #s = v + vdot
    s = (v-0.97531*prev_v)/0.0246901
    states = [lv, d, v, s]
    states = np.array(states).reshape(4,1)
   # print(states)
   # print('.....................')
    scaling  = np.max(np.max(np.matmul(M,states)),0.01) # Prevent division by zero
    umax     = np.min(np.matmul(alpha,states).reshape(len(beta),1)/scaling + beta)
    umin     = np.max(np.matmul(alpha,states).reshape(len(beta),1)/scaling - beta)
    u        = scaling*(umax+umin)/2

    A = [[1.0,0,0,0],[0.0247,1.0,-0.0247,-0.0003],[ 0,0,0.9756,0.0244],[0,0,0.0244,0.9756]]
    B = [0,0.0000,0.0006,0.0494]
    A = np.array(A).reshape(4,4)
    B = np.array(B).reshape(4,1)
    states = np.matmul(A,states.reshape(4,1))+B*u
#    print(states)
#    vn = 0.9756*v+0.0244*s;
 #   input_acceleration = (vn-v)/0.025
    newS = 0.024385*v + 0.975615*states[3] + 0.0493853*u
    input_acceleration = - v + newS
    input_acceleration = (states[2]-v)/0.025
    prev_v = v

    #vi = vi + 13.4498* input_acceleration/42.935
   # vi = vi + 0.3396*input_acceleration
  #  vi = vi + 0.12*input_acceleration
    vi = vi + 0.3396*a[0]*5
    if vi<2.1 and vi>0 and a[0]<0:
        vi = -vi + 0.3396*a[0]
    if vi>-2.1 and vi<0  and a[0]>0:
        vi = -vi + 0.3396*a[0]
   # a.pop(0)
 #   vi = 13.4498* states[2] + 2.14981
    r = 0.065
    if vi >0:
       pwm = 1480 - vi/r
    if vi <0:
       pwm = 1520 - vi/r
    if vi ==0:
       pwm = 1500

    if pwm < 1050:
	pwm = 1050
#    pwm = 1500

#    pwm = 1510
  #  print("goal v",states[2],"actual v",v, 'vi',vi)
    print('actual',vdot,'a[0]',a[0])
    a.pop(0)
#    print(newS,input_acceleration*0.33)
 #   print("measured s",s,"next calc. s",states[3],"actual a",vdot, "input_acceleration",(states[2]-v)/0.025)
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
