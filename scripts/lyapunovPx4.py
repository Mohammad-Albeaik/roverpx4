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
sum_error_R 		= 0
sum_error_L 		= 0
kv 							= 10				# Path planning - Lyapunov controller
kw 							= 20				# Path planning - Lyapunov controller
kp 							= 60				# Speed PID controller
kd 							= 10 				# Speed PID controller
ki 							= 1.5 				# Speed PID controller
dR_Pre 					= 0
dL_Pre 					= 0
omega 					= 0
vel 						= 0



def posCb(msg):
    global x, y, z, roll, pitch, yaw
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    local_ang = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(local_ang)

def velCb(velo):
    global vx,vy,vz, vel

    vx = velo.linear.x
    vy = velo.linear.y
    vz = velo.linear.z
    vel = sqrt(vx*vx + vy*vy + vz*vz)

def angCb(angu):
    global omega

    omega = angu.angular.z


def UpdateSpeed():
    global kv, kw, v, w , velocity, yaw, x ,y, sum_error_L, sum_error_R, kp, ki, kd, dR_Pre, dL_Pre, omega, vel


    xg = 0
    yg = 0
    dx = xg - x 
    dy = yg - y

    rho = sqrt(dx*dx + dy*dy)
    alpha = (atan2(dy,dx)) - yaw
    beta = - alpha - yaw    

    
    v = - kv * (-rho*rho*rho *cos(alpha) + rho*(alpha - beta)*sin(alpha))
    w = kw * alpha

    if v > 1:
        v = 1
    if v < -1:
        v = -1
    if w > 2.25:
        w = 2.25
    if w < -2.25:
        w = -2.25



    if rho < 0.05: 		# 5 cm away from the final destination 
        v = 0
        w = 0


    velocity.linear.x = v
    velocity.angular.z = w

#------------------------------------------------------------------------- Debug -------------------------------------------------------------------------

    #print("x",x)
    #print("y",y)
    #print("yaw",yaw)
    #print("actual",actual)
    #print("alpha",alpha)
    #print("v  ",v)
    #print("w  ",w)
    #print("wr  ",wr)
    #print("wl  ",wl)

#------------------------------------------------------------------------- Debug end-------------------------------------------------------------------------
#------------------------------------------------------------------------- PID Start ----------------------------------------------------------------------
    r = 0.13 # radius 13 cm
    L = 0.33 # length between wheels 33 cm

 
    # vel  		= read actual speed
    # omega 	= read actual speed

    WR = vel/r +L/r *omega 
    WL = 2* vel/r - WR

    # from desired linear and angular velocities, caluclate wheels' rotation rate 
    v = 1.0
    w = 0
    desired_vel = v
    desired_omega = w

    desired_RW = desired_vel/r +L/r * desired_omega 
    desired_LW = 2* desired_vel/r - desired_RW

		
    dR = desired_RW - WR
    dL = desired_LW - WL

    sum_error_R = sum_error_R + dR
    sum_error_L = sum_error_L + dL
    if sum_error_R > 100:
        sum_error_R = 100
    elif sum_error_R < -100:
        sum_error_R = -100
    if sum_error_L > 100:
        sum_error_L = 100
    elif sum_error_L < -100:
        sum_error_L = -100

    error_R = 1500 - (kp * dR + kd * (dR - dR_Pre) + ki * sum_error_R) # PWM signal for the right wheel
    error_L = 1500 - (kp * dL + kd * (dL - dL_Pre) + ki * sum_error_L)  # PWM signal for the left wheel
    
    dR_Pre = dR
    dL_Pre = dL
    dR 	   = error_R
    dl     = error_L

    if error_R > 2100:
        error_R = 2100
    elif error_R < 900:
        error_R = 900
    if error_L > 2100:
        error_L = 2100
    elif error_L < 900:
        error_L = 900

    RcOver.channels = [1500,error_R,1500,error_L,0,0,0,0]   # 4th

    print("error_R",error_R)
    print("error_L",error_L)
    print("v ",v)
    print("actual v ",vel)
    print("w ",w)
    print("actual w ",omega)
    #print("sum_error_L ",sum_error_L)
    #print("sum_error_R ",sum_error_R)
    print("-------------------------------------------------")

#------------------------------------------------------------------------- PID end-------------------------------------------------------------------------


# Main function
def main():

    # Initiate node
    rospy.init_node('Roveer_CU', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(60.0) 

       
    # Subscribe to Rover's local position
    rospy.Subscriber('vrpn_client_node/mmb/pose', PoseStamped, posCb)

    # Subscribe to Rover's local linear velocity
    rospy.Subscriber('mavros/local_position/velocity_local', Twist, velCb)
        # Subscribe to Rover's local angular velocity
    rospy.Subscriber('mavros/local_position/velocity_body', Twist, angCb)

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
    RcOver.channels = [1500,1500,1500,1500,0,0,0,0]   
    rc_pub.publish(RcOver)

if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass
