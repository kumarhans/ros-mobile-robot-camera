#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from random import gauss
import math

def talker():

    Movement = True

    pubMobile = rospy.Publisher('/mobile_robot/cmd_vel', Twist)
    pubCam = rospy.Publisher('/cam_revolute_position_controller/command', Float64)
    pubCamPris = rospy.Publisher('/cam_prismatic_position_controller/command', Float64)

    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    start = rospy.get_time()
    time = 1.2
    switch = 1
    first = True
    t = 0

    while not rospy.is_shutdown():
 
        newt = (t-5) 
        msg = Twist()
        msg.linear.x = 0.1 
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0
        msg.angular.z = 0
        frequency = .7
        upDownAngle = 35/180.0*math.pi

        phase = math.sin(frequency*(newt)*(2*math.pi))/(abs(math.sin(frequency*(newt)*(2*math.pi)))**(.4))
        if t > 5:
            pubMobile.publish(msg)
            msg = Float64()
            msg.data = upDownAngle*phase
            pubCam.publish(msg)
            pubCamPris.publish(msg)
        
        rate.sleep()
        t = t + 1.0/100.0

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass