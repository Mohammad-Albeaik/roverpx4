#!/usr/bin/env python

# ROS python API
import rospy
# Joy message structure
from sensor_msgs.msg import Joy
# 3D point & Stamped Pose msgs & Orientation as quaternion
from geometry_msgs.msg import Point, PoseStamped, Quaternion
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
#import math for arctan and sqrt function
from math import atan2, sqrt, pi
#import quaternion transformation
from tf.transformations import euler_from_quaternion
from numpy import array

from std_msgs.msg import String

# Flight modes class
# Flight modes are activated using ROS services


kp = 300
kpo = 200
speed = 1500
delta = 0


class fcuModes:
        def __init__(self):
                pass

        def setArm(self):
                rospy.wait_for_service('mavros/cmd/arming')
                try:
                        armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
                        armService(True)
                except rospy.ServiceException, e:
                        print "Service arming call failed: %s"%e

        def setDisarm(self):
                rospy.wait_for_service('mavros/cmd/arming')
                try:
                        armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
                        armService(False)
                except rospy.ServiceException, e:
                        print "Service disarming call failed: %s"%e

        def setStabilizedMode(self):
                rospy.wait_for_service('mavros/set_mode')
                try:
                        flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
                        flightModeService(custom_mode='STABILIZED')
                except rospy.ServiceException, e:
                        print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

        def setOffboardMode(self):
                rospy.wait_for_service('mavros/set_mode')
                try:
                        flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
                        flightModeService(custom_mode='OFFBOARD')
                except rospy.ServiceException, e:
                        print "service set_mode call failed: %s. Offboard Mode could not be set."%e

        def setAltitudeMode(self):
                rospy.wait_for_service('mavros/set_mode')
                try:
                        flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
                        flightModeService(custom_mode='ALTCTL')
                except rospy.ServiceException, e:
                        print "service set_mode call failed: %s. Altitude Mode could not be set."%e

        def setPositionMode(self):
                rospy.wait_for_service('mavros/set_mode')
                try:
                        flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
                        flightModeService(custom_mode='POSCTL')
                except rospy.ServiceException, e:
                        print "service set_mode call failed: %s. Position Mode could not be set."%e

        def setAutoLandMode(self):
                rospy.wait_for_service('mavros/set_mode')
                try:
                        flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
                        flightModeService(custom_mode='AUTO.LAND')
                except rospy.ServiceException, e:
                        print "service set_mode call failed: %s. Autoland Mode could not be set."%e

# Main class: Converts joystick commands to position setpoints
class Controller:
        # initialization method
        def __init__(self):
                # Drone state
                self.state = State()

                # Instantiate a rc signal message
                self.rc = OverrideRCIn()
                self.rc.channels = [1500,1500,1500,1500,1500,1500,1500,1500]
                # Instantiate a joystick message
                self.joy_msg        = Joy()
                # initialize
                self.joy_msg.axes=[0.0, 0.0, 0.0]

                # Step size for position update
                self.STEP_SIZE = 2.0

                # Fence. We will assume a square fence for now
                self.FENCE_LIMIT = 1.0

                # A Message for the current local position of the drone
                self.local_pos = Point(0.0, 0.0, 0.0)

                # A Message for the current local orientation of the drone
                self.local_ang = [0.0,0.0,0.0,0.0]
                self.ang = 0

                self.modes = fcuModes()

                #Debugging parameters
                self.dbg = str()
                self.joy_dbg = 0.0
                self.yaw_dbg = 0.0

        # Callbacks

        ## local position callback
        def posCb(self, msg):
                self.local_pos.x = msg.pose.position.x
                self.local_pos.y = msg.pose.position.y
                self.local_pos.z = msg.pose.position.z





                #quaternion is set explicitly in this form in order to use euler_from_quaternion() function later
                self.local_ang = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

        ## joystick callback
        def joyCb(self, msg):
                self.joy_msg = msg

                # # If button 1 on joystick is pressed
                # if msg.buttons[0] > 0:
                #        self.modes.setArm()

                # # If button 2 on joystick is pressed
                # if msg.buttons[1] > 0:
                #         self.modes.setAutoLandMode()

                # # If button 3 on joystick is pressed
                # if msg.buttons[2] > 0:
                #        self.modes.setOffboardMode()

                # # If button 11 on joystick is pressed
                # if msg.buttons[10] > 0:
                #        self.modes.setDisarm()


        ## Drone State callback
        def stateCb(self, msg):
                self.state = msg
        
        ## Finds relative angle between robot and joystick:
        def calcAngle(self):
                #Check that joystick x axis is not zero, in order to avoid divide by zero error
                if self.joy_msg.axes[0] != 0:
                        joy_ang = atan2(-self.joy_msg.axes[1],-self.joy_msg.axes[0])

                #If joystick x is zero set explicitly
                else:
                        if self.joy_msg.axes[1] == 0:   #if both zero then no action required
                                joy_ang = 0.0
                        elif -self.joy_msg.axes[1] > 0: #if y axis is positive then set to 90 degrees
                                joy_ang = pi/2.0
                        else:                           #if y axis is negative then set to -90 degrees
                                joy_ang = -pi/2.0
                
                roll, pitch, yaw = euler_from_quaternion(self.local_ang)
                self.joy_dbg = joy_ang
                self.yaw_dbg = yaw
                return  joy_ang - yaw

        ## Finds magnitude of thrust:
        def calcMag(self):
                return (100*sqrt(self.joy_msg.axes[0]**2+self.joy_msg.axes[1]**2)+1500) 


        def updateRCOveride(self):
                        global kp, kpo, speed, delta
                        #thrust
                        dx = -0.3-self.local_pos.x
                        dy = -0.2-self.local_pos.y
                        desired_orientation = (atan2(dy,dx))/pi
                        error_ds = sqrt(dx*dx +dy*dy)

                        error_or = ((float(self.local_ang[3]) - float(desired_orientation)))


                        if error_ds >0.1:
                           speed = 1500 + error_ds*kp                           
                           if speed>1700:
                           		speed = 1700
                           if speed <1500:
                           		speed = 1500

                        if abs(error_or) >0.02:
                           delta = error_or*kpo
															
                        if delta >150:
                           delta = 150
                        if delta <-150:
                           delta = -150 
													 
                        if delta > 0:
                           Right = speed + delta
                           Left  = speed 
                        else:
                           Right = speed 
                           Left  = speed + delta
													 
													 
                        self.rc.channels = [1500,1200,1500,0,0,0,0,0] #move forward proportional to joystick magnitude           1&4
                           




# Main function
def main():

        # Initiate node
        rospy.init_node('optitrackcontrol', anonymous=True)

        # controller object
        cnt = Controller()

        # ROS loop rate, [Hz]
        rate = rospy.Rate(90.0)

        # Subscribe to drone state
        rospy.Subscriber('mavros/state', State, cnt.stateCb)

        # Subscribe to drone's local position
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

        # subscribe to joystick topic
        #rospy.Subscriber('joy', Joy, cnt.joyCb)

        # RCOveride publisher
        rc_pub = rospy.Publisher('mavros/rc/override',OverrideRCIn, queue_size=1)

        #DEBUG
        #debug_pub = rospy.Publisher('debug/state',String, queue_size=1)

        # Some lines deleted from SITL code


        # ROS main loop
        while not rospy.is_shutdown():

                cnt.updateRCOveride()
                rc_pub.publish(cnt.rc)
               # debug_pub.publish(cnt.dbg)      #debugging
                rate.sleep()


if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass
