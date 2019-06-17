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
kv = 3000
kw = 30
kp = 1
kd = 0.35
ki = 0.5
dxPre = 0
dyPre = 0



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
    #xg = -1
    #yg = -1
    #dx = xg - x 
    #dy = yg - y
    #sumx_error = sumx_error + dx 
    #sumy_error = sumy_error + dy
    #if sumx_error > 1:
    #    sumx_error = 1
    #elif sumx_error < -1:
    #    sumx_error = -1
    #if sumy_error > 1:
    #    sumy_error = 1
    #elif sumy_error < -1:
    #    sumy_error = -1

    #errorx = kp * dx + kd * (dx - dxPre) + ki * sumx_error 
    #errory = kp * dy + kd * (dy - dyPre) +ki * sumy_error
    
    #dxPre = dx
    #dyPre = dy
    #dx = errorx
    #dy = errory

    #actual = sqrt(dxPre*dxPre + dyPre*dyPre)
    #if actual < 0.05:
    #    v = 0
    #    w = 0
#------------------------------------------------------------------------- PID end-------------------------------------------------------------------------
    xg = -1
    yg = -1
    dx = xg - x 
    dy = yg - y

    rho = sqrt(dx*dx + dy*dy)
    alpha = (atan2(dy,dx)) - yaw
    beta = - alpha - yaw    

    
    v = - kv * (-rho*rho*rho *cos(alpha) + rho*(alpha - beta)*sin(alpha))
    w = kw * alpha

    if v>1500:
        v = 1500
    if v<-1500:
        v = -1500
    if w>30:
        w = 30
    if w<-30:
        w = -30



    if rho < 0.05:
        v = 0
        w = 0


    velocity.linear.x = v
    velocity.angular.z = w


#------------------------------------------------------------------------- Debug -------------------------------------------------------------------------
    wr = v/13 +33/13 *w 
    wl = 2* v/13 -wr
                                                     

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



    # ROS main loop
    while not rospy.is_shutdown():

            UpdateSpeed()
            speed_pub.publish(velocity)
            rate.sleep()


if __name__ == '__main__':
    try:
            main()
    except rospy.ROSInterruptException:
            pass
