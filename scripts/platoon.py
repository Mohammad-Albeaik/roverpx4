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
from sensor_msgs.msg import Joy
import numpy as np
from numpy import linalg as n

x         			= 0.0
y         			= 0.0
z         			= 0.0
local_ang 			= [0.0, 0.0, 0.0, 0.0]
roll      			= 0.0
pitch     			= 0.0
yaw       			= 0.0
RcOver 					= OverrideRCIn()
RcOver.channels = [1500,1500,1500,1500,0,0,0,0]
sum_error_rho 		= 0
sum_error_alpha 		= 0
kvp 							= 20				## Speed PID controller
kvd                             = 2                 # Speed PID controller
kvi                             = 0.05                 # Speed PID controller
kwp 							= 50				# Speed PID controller
kwd 							= 2					# Speed PID controller
kwi 							= 0.05 				# Speed PID controller
old_rho 					    = 0
old_alpha 					    = 0
xg 						= 0
yg 						= 0
x_leader                = 0
y_leader                = 0
memory = []   #
memory.append([x,y]) # memory[0] stores the position of the rover 
d = np.zeros(shape = (2,2), dtype=np.int)

def posCb(msg):
    global x, y, z, roll, pitch, yaw
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    local_ang = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(local_ang)

def posLeaderCb(msg)
    x_leader = msg.pose.position.x
    y_leader = msg.pose.position.y



def UpdateSpeed():
    global v, w , velocity, yaw, x ,y, sum_error_rho, sum_error_alpha, kvp, kwp, kvd, kwd, kvi, kwi, old_rho, old_alpha, xg, yg

#------------------------------------------------------------------------- PID Start ----------------------------------------------------------------------

#------------------------------------------------ info from the leader



    memory[0]=[x,y] # memory[0] stores the position of the rover 


    d[0] = [x_leader,y_leader]       # the position of the leader
    d[1] = memory[-1]    # last stored position

    distance  = n.norm(np.diff(d, axis=0),2) # norm2 of [dx,dy]

    if distance > 0.4: # threshhold: distance between stored points
        memory.append([x_leader,y_leader])

    # finding how far are we from the next waypoint
    d[0] = [x,y]       # the position of the rover
    d[1] = memory[1]    # next goal position
    nextgoal = n.norm(np.diff(d, axis=0),2) # difference norm2 of [dx,dy]

    if nextgoal < 0.2:
        del memory[1]
        xg,yg = memory[1]

    # calculating rho: the actual distance from the robot to the 
    rho = sum(n.norm(np.diff(memory, axis=0),2,axis=1))



#------------------------------------------------- IFTL end



    # error in xy plain 
    dx = 0.2 - rho
    dy = yg - y


    angle = atan2(dy,dx)
    alpha = angle - yaw

    if alpha > 5:
       alpha = alpha - 2*pi
    elif alpha < -5:
       alpha = alpha + 2*pi

    sum_error_rho = sum_error_rho + rho
    sum_error_alpha = sum_error_alpha + alpha


    if sum_error_rho > 10:
        sum_error_rho = 10
    elif sum_error_rho < -10:
        sum_error_rho = -10


    if sum_error_alpha > 10:
        sum_error_alpha = 10
    elif sum_error_alpha < -10:
        sum_error_alpha = -10


    v =  kvp * rho   + kvd * (abs(rho) - abs(old_rho)) + kvi * sum_error_rho
    w =  kwp * alpha + kwd * (abs(alpha) - abs(old_alpha)) + kwi * sum_error_alpha
    

    old_rho     = rho
    old_alpha   = alpha



    

    # try not to exceed these valuse so that WR and WL are within the allowable range (900-2100)
    if v > 30:
        v = 30
    if v < -30:
        v = -30
    if w > 90:
        w = 90
    if w < -90:
        w = -90



    if abs( rho) < 0.1: 		# stop if you are 5 cm away from the final destination 
        v = 0
        w = 0


#------------------------------------------------------------------------- PID end-------------------------------------------------------------------------
    r = 0.13 # radius 13 cm
    L = 0.33 # length between wheels 33 cm


    WR =   v/r +L/r * w
    WL =   2* v/r - WR

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


	#------------------------------------------------------------------------- Debug -------------------------------------------------------------------------

    print("x",x)
    print("y",y)
    print("yaw",yaw)
    #print("alpha",alpha)
    #print("v  ",v)
    #print("w  ",w)
    print("WR  ",WR)
    print("WL  ",WL)
    print("v ",v)
    print("w ",w)
    #print("sum_error_L ",sum_error_L)
    #print("sum_error_R ",sum_error_R)
    print("-------------------------------------------------")
#------------------------------------------------------------------------- Debug end-------------------------------------------------------------------------



# Main function
def main():

    # Initiate node
    rospy.init_node('Roveer_CU', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(80.0)

    # Subscribe to Rover's local position
    rospy.Subscriber('vrpn_client_node/mmb/pose', PoseStamped, posCb)


    # Subscribe to Rover's local position
    rospy.Subscriber('vrpn_client_node/leader/pose', PoseStamped, posLeaderCb)


    # RCOveride publisher
    rc_pub = rospy.Publisher('rover1/mavros/rc/override',OverrideRCIn, queue_size=1)


    # ROS main loop
    while not rospy.is_shutdown():

            UpdateSpeed()
            speed_pub.publish(velocity)
            rc_pub.publish(RcOver)
            rate.sleep()

 

if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass
