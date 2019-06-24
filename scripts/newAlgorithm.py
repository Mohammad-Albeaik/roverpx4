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

x         = 0.0
y         = 0.0
z         = 0.0
local_ang = [0.0, 0.0, 0.0, 0.0]
roll      = 0.0
pitch     = 0.0 
yaw       = 0.0
velocity = Twist()
sumx_error = 0
sumy_error = 0
kv = 50
kw = 100
kp = 1
kd = 0.5
ki = 0.0001        #was 0.000075
dxPre = 0
dyPre = 0
RcOver   = OverrideRCIn()
RcOver.channels    =[1500,1500,1500,1500,0,0,0,0]



def posCb(msg):
    global x, y, z, roll, pitch, yaw
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    local_ang = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(local_ang)




def UpdateSpeed():
    global kv, kw, v, w , velocity, yaw, x ,y, sumy_error, sumx_error, kp, ki, kd, dxPre, dyPre

#------------------------------------------------------------------------- PID -------------------------------------------------------------------------
    xg = 1
    yg = 2
    dx = xg - x 
    dy = yg - y
    sumx_error = sumx_error + dx 
    sumy_error = sumy_error + dy
    if sumx_error > 1500:
        sumx_error = 1500
    elif sumx_error < -1500:
        sumx_error = -1500
    if sumy_error > 1500:
        sumy_error = 1500
    elif sumy_error < -1500:
        sumy_error = -1500

    errorx = kp * dx + kd * (dx - dxPre) + ki * sumx_error 
    errory = kp * dy + kd * (dy - dyPre) +ki * sumy_error


    dxPre = dx
    dyPre = dy
    dx = errorx
    dy = errory

    actual = sqrt(dxPre*dxPre + dyPre*dyPre)

#------------------------------------------------------------------------- PID end-------------------------------------------------------------------------
    #xg = 1
    #yg = 2
    #dx = xg - x 
    #dy = yg - y

    rho = sqrt(dx*dx + dy*dy)
    alpha = (atan2(dy,dx)) - yaw
    beta = - alpha - yaw    


    v = - kv * (-rho*rho*rho *cos(alpha) + rho*(alpha - beta)*sin(alpha))
    w = kw * alpha

    if v>25:
        v = 25
    if v<-25:
        v = -25
    if w>60:
        w = 60
    if w<-60:
        w = -60



   # if rho < 0.05:
    #    v = 0
     #   w = 0
    if actual < 0.2:
       v = 0
       w = 0

    velocity.linear.x = v
    velocity.angular.z = w


#------------------------------------------------------------------------- Debug -------------------------------------------------------------------------
    r = 0.13
    L = 0.33
    wr = v/r + L/r *w
    wl = 2* v/r -wr

    if wr > 0:
       wr = -wr + 1405
    elif wr < 0:
       wr = - wr + 1595
    if wl >0:
       wl = -wl + 1405
    elif wl<0:
       wl = -wl + 1595

    RcOver.channels = [1500,wr,1500,wl,0,0,0,0]

    print("x",x)
    print("y",y)
    #print("yaw",yaw)
    print("actual",actual)
    print("summx_error",sumx_error)
    print("v  ",v)
    print("w  ",w)
    #print("wr  ",wr)
    #print("wl  ",wl)

#------------------------------------------------------------------------- Debug end-------------------------------------------------------------------------



# Main function
def main():

    # Initiate node
    rospy.init_node('Roveer_CU', anonymous=True)

    # ROS loop rate, [Hz]
    rate = rospy.Rate(80.0) # at higher rate, the topic overflow and causes bad control (delay)


    # Subscribe to Rover's local position
    rospy.Subscriber('vrpn_client_node/mmb/pose', PoseStamped, posCb)


    # Speed publisher
    speed_pub = rospy.Publisher('velocity_cm',Twist, queue_size=1)   # change to velocity_cm

    # RcOverride
    rc_pub = rospy.Publisher('mavros/rc/override',OverrideRCIn,queue_size=1)

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
