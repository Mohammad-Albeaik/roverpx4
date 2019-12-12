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
from mavros_msgs.msg import OverrideRCIn,RCIn
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
states = [0,0,0]
states = np.array(states).reshape(3,1)
Ti = time.time()
input_acceleration = 0
z                               = 0.0
local_ang                       = [0.0, 0.0, 0.0, 0.0]
roll                            = 0.0
pitch                           = 0.0
yaw                             = 0.0
wi = 0
d = 0

#............................ agent's callback functions 

def posCb(msg):
    global x, y, yaw, roll, pitch, local_ang
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    local_ang = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(local_ang)


def vCb(msg):
    global v,w
    xdot    = msg.linear.x
    ydot    = msg.linear.y
    forward = msg.linear.z
    w = msg.angular.z
    v = forward*sqrt(xdot*xdot+ydot*ydot)
    v = msg.angular.x

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
    allmat  = scipy.io.loadmat('../catkin_ws/src/roverpx4/scripts/1v10hz.mat')
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
    global x, y, v, w, vdot, lx, ly, lv, lw, M, alpha, beta, u, prev_v, prev_s, states, Ti, A,B,input_acceleration,d,s

    T = time.time()
    dT = T - Ti
    Ti = T

    desired_distance = 0.75
    d = sqrt((x-lx)*(x-lx) + (y-ly)*(y-ly)) - desired_distance

    states = [lv, d, v]
    states = np.array(states).reshape(3,1)

    scaling  = np.max(np.max(np.matmul(M,states)),0.01) # Prevent division by zero
    umax     = np.min(np.matmul(alpha,states).reshape(len(beta),1)/scaling + beta)
    umin     = np.max(np.matmul(alpha,states).reshape(len(beta),1)/scaling - beta)
    u        = scaling*(umax+umin)/2

    states = np.matmul(A,states.reshape(3,1))+B*u

    input_acceleration = u
    prev_v = v



def UpdateAcceleration():
    global input_acceleration, vi, states,x,y,lx,ly,v,T0, yaw, wi,d,s, lv, summ_error

 #   vi = vi + 0.3396*input_acceleration
   # vi = vi + 0.3396*input_acceleration
    T = time.time()

#    states[2] = -1.1
    if states[2]>=0:
#      	vi = 13.28* states[2] + 2.3
#       vi = 13.7041*states[2] +1.66759
#       vi = 13.6041*states[2] +1.66759
       vi = 1.65278 + 10.6944 * states[2]
    if states[2]<-0.004:
#    	vi = 13.25* states[2] - 2.33
#       vi = 12.4242 * states[2] -0.812545
       vi = 10.5485 * states[2] -0.551178
    L = 0.33 # length between wheels 33 cm
    r = 0.065


    if vi >50:
        vi = 50
    if vi <-50:
        vi = -50

    if input_acceleration > 6:
       input_acceleration = 6
    if input_acceleration < -6:
       input_acceleration = -6


    vi = vi + 0.65*input_acceleration

    if vi >50:
       vi = 50
    if vi <-50:
       vi = -50


    kg = 80
    kh = 50
    gamma1 = -kg * x;
    des_ang = -pi/2
    gamma2 = kh * (des_ang - yaw);
    if vi >0:
       wi = gamma1 + gamma2;
    if vi <0:
       wi = -gamma1 + gamma2
  #  alpha = (-pi/2) - yaw
  #  wi = 70*alpha

    if wi >20:
        wi = 20
    if wi <-20:
        wi = -20




    WR =   vi/r +L/r * wi
    WL =   2* vi/r - WR

    if WR > 0:
        WR = - WR + 1480
    elif WR <0:
        WR = - WR +1520
    else:
        WR = 1500
    if WL > 0:
        WL = - WL + 1480 - 10
    elif WL < 0:
        WL = - WL +1520 +10
    else:
        WL = 1500

    if (y>3.3 and vi <0) or (y<-3.3 and vi >0):
       WL = 1500
       WR = 1500
    if abs(x) >1:
       WL = 1500
       WR = 1500
#    print(v,vi)
#    print(lv,d,v,s,states[2], input_acceleration)
#    print(WR,WL)
#    print(sqrt((x-lx)*(x-lx) + (y-ly)*(y-ly)) - 0.75)
    RcOver.channels = [1500, WR,1500, WL,0,0,0,0]   # 4th

def speedC():
    global states,vi,wi,yaw,x,v,input_acceleration, T0


 #   if abs(vi)<1.66 and (input_acceleration >0):
 #      vi = 1.66
 #   if abs(vi)<1.66 and (input_acceleration <0):
 #      vi = -1.66

    if time.time()-T0<10:
       input_acceleration = 0.3
    elif time.time()-T0<17:
       input_acceleration = -0.1
    else:
       input_acceleration = 1.5

    kp = 0.1

    T = time.time() - T0

    e = input_acceleration - v
    vi = vi + kp * e

    if vi >50:
       vi = 50
    if vi <-50:
       vi = -50

    L = 0.33 # length between wheels 33 cm
    r = 0.065

    kg = 80
    kh = 50
    gamma1 = -kg * x;
    des_ang = -pi/2
    gamma2 = kh * (des_ang - yaw);
    if vi >0:
       wi = gamma1 + gamma2;
    if vi <0:
       wi = -gamma1 + gamma2


    if wi >20:
        wi = 20
    if wi <-20:
        wi = -20




    WR =   vi/r +L/r * wi
    WL =   2* vi/r - WR

    if WR > 0:
        WR = - WR + 1480
    elif WR <0:
        WR = - WR +1520
    else:
        WR = 1500
    if WL > 0:
        WL = - WL + 1480 - 10
    elif WL < 0:
        WL = - WL +1520 +10
    else:
        WL = 1500

    if (y>3.3 and vi <0) or (y<-3.3 and vi >0):
       WL = 1500
       WR = 1500
    if abs(x) >1:
       WL = 1500
       WR = 1500

#    print(v,T,input_acceleration)
    print(lv,d,v,input_acceleration,states[2])
    RcOver.channels = [1500, WR,1500, WL,0,0,0,0]   # 4th

# Main function
def main():
    global T0

    time.sleep(4)
    T0 = time.time()
    # Initiate node
    rospy.init_node('jeffController', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(10.0)

    # Subscribe to leader's local position
    rospy.Subscriber('vrpn_client_node/'+ rospy.get_param('leader') +'/pose', PoseStamped, lposCb) 
    # Subscribe to leader's local velocity
    rospy.Subscriber(rospy.get_param('leader')+'/velocity',Twist, lvCb)

    # Subscribe to your local position
    rospy.Subscriber('vrpn_client_node/' + rospy.get_param('rover') + '/pose', PoseStamped, posCb)
    # Subscribe to your local velocity
    rospy.Subscriber(rospy.get_param('rover') +'/velocity',Twist, vCb)
    # Subscribe to your local acceleration
    rospy.Subscriber(rospy.get_param('rover') + '/acceleration',Twist, acCb)


    rc_pub = rospy.Publisher('mavros/rc/override',OverrideRCIn, queue_size=1)

    ControllerMatrices()
    counter = 0

    # ROS main loop
    while not rospy.is_shutdown():
            counter = counter +1
            if counter%1 ==0:
              controller()
              #UpdateAcceleration()
              speedC()
            rc_pub.publish(RcOver)
            rate.sleep()


if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            print("NODE KILLED")
