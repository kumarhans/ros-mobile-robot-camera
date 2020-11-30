#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from random import gauss
import math

def swingerDynamics(t):
    t = t- 10
    frequency = .3
    upDownAngle = 20.0/180.0*math.pi
    xVel = .2 #m/s
    phase = math.sin(frequency*t*(2*math.pi))


    msg = Twist()
    msg.angular.x = 0.0
    msg.angular.y = upDownAngle*phase
    msg.angular.z = 0.0
    msg.linear.x = t*xVel
    msg.linear.y = 0.0
    msg.linear.z = 1.25  
    return msg,phase


def hopperDynamics(t):
    frequency = .5
    upDownAngle = 20.0/180.0*math.pi
    xVel = .2 #m/s
    phase = math.sin(frequency*t*(2*math.pi))


    msg = Twist()
    msg.angular.x = 0.0
    msg.angular.y = 0.5#upDownAngle*phase
    msg.angular.z = 0.0
    msg.linear.x = t*xVel
    msg.linear.y = 0.0
    msg.linear.z = 1.25  +phase/2
    return msg,phase

def hopperSwingDynamics(t):
    frequency = .5
    upDownAngle = 25.0/180.0*math.pi
    xVel = .2 #m/s
    phase = math.sin(frequency*t*(2*math.pi))


    msg = Twist()
    msg.angular.x = 0.0
    msg.angular.y = -upDownAngle*phase
    msg.angular.z = 0.0
    msg.linear.x = t*xVel
    msg.linear.y = 0.0
    msg.linear.z = 1.25  +phase/2
    return msg,phase



def talker():

    Movement = True

    pubPos = rospy.Publisher('/cmd_pos', Twist)
    pubPhase = rospy.Publisher('/cmd_phase', Float64)
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(100) # 100hz

    start = rospy.get_time()
    first = True
    
    t = 0
    

    while not rospy.is_shutdown():

        if t < 10:
             
            msg = Twist()
            msg.angular.x = 0.0
            msg.angular.y = .5
            msg.angular.z = 0.0
            msg.linear.x = 0.0 
            msg.linear.y = 0.0
            msg.linear.z = 1.25  
            pubPos.publish(msg)
             
        else:
            msg,phase = swingerDynamics(t)
            #msg,phase = hopperDynamics(t)
            #msg,phase = hopperSwingDynamics(t)
            pubPos.publish(msg)
            pubPhase.publish(phase)


        rate.sleep()
        t = t + 1.0/100.0

        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass