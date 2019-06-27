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


x         			= 0.0
y         			= 0.0
z         			= 0.0
local_ang 			= [0.0, 0.0, 0.0, 0.0]
roll      			= 0.0
pitch     			= 0.0
yaw       			= 0.0
velocity 				= Twist()
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
joy_msg   					= Joy()
joy_msg.axes 					= [0.0,0.0,0.0]
xg 						= 0
yg 						= 0


def posCb(msg):
    global x, y, z, roll, pitch, yaw
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    local_ang = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(local_ang)



def UpdateSpeed():
    global v, w , velocity, yaw, x ,y, sum_error_rho, sum_error_alpha, kvp, kwp, kvd, kwd, kvi, kwi, old_rho, old_alpha, xg, yg


    xg = 1.5
    yg = 1.68

#------------------------------------------------- Distance & linear velocity control ----------------------------------------------------------------

    dx = xg - x
    dy = yg - y
    rho = sqrt(dx*dx + dy*dy)
    


    sum_error_rho = sum_error_rho + rho

    if sum_error_rho > 10:
        sum_error_rho = 10
    elif sum_error_rho < -10:
        sum_error_rho = -10

    v =  kvp * rho   + kvd * (abs(rho) - abs(old_rho)) + kvi * sum_error_rho

    if v > 30:
        v = 30
    if v < -30:
        v = -30

    old_rho     = rho

#----------------------------------------------------- Orientation control (path following)--------------------------------------------------------------
    dx = 0.00001;
    y2 = 0.75*(x+dx)*(x+dx);
    dy = y2 - y;

    alpha = (atan2(dy,dx)) - yaw

    # if (alpha < pi/2) and (alpha > -pi/2):
    #     rho = rho
    # else:
    #     rho = - rho

    sum_error_alpha = sum_error_alpha + alpha
    if sum_error_alpha > 10:
        sum_error_alpha = 10
    elif sum_error_alpha < -10:
        sum_error_alpha = -10

    w =  kwp * alpha + kwd * (abs(alpha) - abs(old_alpha)) + kwi * sum_error_alpha

    if w > 90:
        w = 90
    if w < -90:
        w = -90

    old_alpha   = alpha

#-------------------------------------------------------------- END --------------------------------------------------------------


    if abs( rho) < 0.1:         # 5 cm away from the final destination 
        v = 0
        w = 0

    velocity.linear.x = v
    velocity.angular.z = w

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

def joyCB(msg):
	global xg,yg
	xg = msg.axes[1]*2.5
	yg = msg.axes[0]*2.5


def StopBeforeKilling():
    for i in range(1,120):
        # Stop the rover before killiing the node. 
        RcOver.channels = [1500,1500,1500,1500,0,0,0,0]   
        rc_pub.publish(RcOver)
        rate.sleep()

# Main function
def main():

    # Initiate node
    rospy.init_node('Roveer_CU', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(60.0)

    # Subscribe to Rover's local position
    rospy.Subscriber('vrpn_client_node/mmb/pose', PoseStamped, posCb)

    # Subscribe to joystick topic
    rospy.Subscriber('joy',Joy,joyCB)

    # Speed publisher
    speed_pub = rospy.Publisher('velocity_cm',Twist, queue_size=1)   # change to velocity_cm

    # RCOveride publisher
    rc_pub = rospy.Publisher('mavros/rc/override',OverrideRCIn, queue_size=1)


    # ROS main loop
    while not rospy.is_shutdown():

            UpdateSpeed()
            speed_pub.publish(velocity)
            rc_pub.publish(RcOver)
            rate.sleep()

    # Stop the rover before killiing the node. 
    StopBeforeKilling()

if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass
