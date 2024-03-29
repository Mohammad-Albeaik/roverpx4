#!/usr/bin/env python
# ROS python API
import rospy
# Joy message structure
# 3D point & Stamped Pose msgs & Orientation as quaternion
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist, TwistStamped
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
sum_error_v 		= 0
sum_error_w 		= 0
kv 							= 30				# Path planning - Lyapunov controller
kw 							= 6				# Path planning - Lyapunov controller
kvp 							= 60				# Speed PID controller
kvd 							= 5 				# Speed PID controller
kvi 						        = 0.5 				# Speed PID controller
kwp                      				= 90
kwd							= 10# 0.5
kwi							= 1
dv_Pre 					= 0
dw_Pre 					= 0
omega 					= 0
vel 						= 0
right						= 0
left 						= 0
imV 						= 0
WR 						= 0
WL 						= 0

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

    omega = angu.twist.angular.z


def UpdateSpeed():
    global kv, kw, v, w , velocity, yaw, x ,y, sum_error_w, sum_error_v, kvp, kvi, kvd,kwp, kwd, kwi, dv_Pre, dw_Pre, omega, vel, right, left, imV, WR, WL


    xg = 0
    yg = 0
    dx = xg - x
    dy = yg - y

    rho = sqrt(dx*dx + dy*dy)
    alpha = (atan2(dy,dx)) - yaw
    beta = - alpha - yaw


    v = - kv * (-rho*rho*rho *cos(alpha) + rho*(alpha - beta)*sin(alpha))
    w = kw * alpha

    if v > 0.9:
        v = 0.9
    if v < -0.9:
        v = -0.9
    if w > 2.55:
        w = 2.55
    if w < -2.55:
       w = -2.55



    if rho < 0.05: 		# 5 cm away from the final destination 
        v = 0
        w = 0


    velocity.linear.x = v
    velocity.angular.z = w

#------------------------------------------------------------------------- Debug -------------------------------------------------------------------------

   # print("x",x)
   # print("y",y)
    #print("yaw",yaw)
    #print("actual",actual)
    #print("alpha",alpha)
   # print("v  ",v)
   # print("w  ",w)
    #print("wr  ",wr)
    #print("wl  ",wl)

#------------------------------------------------------------------------- Debug end-------------------------------------------------------------------------
#------------------------------------------------------------------------- PID Start ----------------------------------------------------------------------
    r = 0.13 # radius 13 cm
    L = 0.33 # length between wheels 33 cm

    # need to find velocity out of the speed and previous WR and WL 
    if WR>1594:
       right = WR - 1595
    elif WR <1406:
       right = WR - 1405
    if WL>1594:
       left = WL - 1595
    elif WL <1406:
       left = WL - 1405
    imV = (right + left)*r/2
    if imV>0:       # this means PWMs were above 1500... robot moving backward
       vel = - abs(vel)
    elif imV<0:     # this means PWMs were bellow 1500... robot moving forward 
       vel = abs( vel)
    # from desired linear and angular  velocities
  #  v = 0.0
  #  w = 2.0


    dv = v - vel
    dw = w - omega

    sum_error_v = sum_error_v + dv
    sum_error_w = sum_error_w + dw

    if sum_error_v > 500:
        sum_error_v = 500
    elif sum_error_v < -500:
        sum_error_v = -500
    if sum_error_w > 500:
        sum_error_w = 500
    elif sum_error_w < -500:
        sum_error_w = -500

    error_v = - (kvp * dv + kvd * (dv - dv_Pre) + kvi * sum_error_v) # PWM signal for the right wheel
    error_w = - (kwp * dw + kwd * (dw - dw_Pre) + kwi * sum_error_w)  # PWM signal for the left wheel

    if error_v > 45 :
       error_v = 45
    elif error_v < -45:
       error_v = -45
    if error_w > 100:
       error_w = 100
    elif error_w <-100:
       error_w = -100

    dv_Pre = dv
    dw_Pre = dw
    dv 	   = error_v
    dw     = error_w

    WR =  error_v/r + L/r * error_w
    WL =  2 * error_v/r - WR

   # WR = WR + 1500
   # WL = WL + 1500

    if WR > 0:
        WR = WR + 1595
    elif WR < 0:
        WR = WR +1405
    if WL > 0:
        WL = WL + 1595
    elif WL < 0:
        WL = WL + 1405

    RcOver.channels = [1500,WR,1500,WL,0,0,0,0]   # 4th

    print("yaw ",yaw)
    #print("desired w ",w)
    print("dv error ",dv)
    print("dw error ",dw)
    print("actual w ",omega)
    print("actual v ",vel)
    print("PWM R ",WR)
    print("PWM L ",WL)
    print("-------------------------------------------------")

#------------------------------------------------------------------------- PID end-------------------------------------------------------------------------


# Main function
def main():

    # Initiate node
    rospy.init_node('Roveer_CU', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(80.0)

    # Subscribe to Rover's local position
    rospy.Subscriber('vrpn_client_node/mmb/pose', PoseStamped, posCb)

    # Subscribe to Rover's local linear velocity
    rospy.Subscriber('velocity', Twist, velCb)
        # Subscribe to Rover's local angular velocity
    rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, angCb)

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
    rate.sleep()
    rc_pub.publish(RcOver)

if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass
