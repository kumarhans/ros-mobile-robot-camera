#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.srv import SetModelState

from gazebo_msgs.msg import LinkStates, ModelState
from random import gauss
import math

import roslaunch









def gaz_callback(msgs):
    if msgs.pose[11].position.x > 10.0 and msgs.pose[11].position.x < 10.1:

        state_msg = ModelState()
        state_msg.model_name = 'mobile_robot'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.7
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        t = 0
       

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e



def talker():

    Movement = True

    pubMobile = rospy.Publisher('/mobile_robot/cmd_vel', Twist)
    pubCam = rospy.Publisher('/cam_revolute_position_controller/command', Float64)
    pubCamPris = rospy.Publisher('/cam_prismatic_position_controller/command', Float64)
<<<<<<< HEAD
    rospy.Subscriber("/gazebo/link_states",LinkStates, gaz_callback)
=======
    pubPhase = rospy.Publisher('/mobile_robot/phase', Float64)

>>>>>>> 99d679d79a0c0a781e555b51db3559d3dfc8dedc

    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    start = rospy.get_time()
    time = 1.2
    switch = 1
    first = True
    t = 0

    package = 'rqt_gui'
    executable = 'rqt_gui'
    node = roslaunch.core.Node(package, executable)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    print(process.is_alive())
    #process.stop()

    while not rospy.is_shutdown():
 
        newt = (t-5) 
        msg = Twist()
        msg.linear.x = 0.2 
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0
        msg.angular.z = 0
        frequency = .7
        upDownAngle = 25/180.0*math.pi
        heaveLength = .3

        phase = math.sin(frequency*(newt)*(2*math.pi))/(abs(math.sin(frequency*(newt)*(2*math.pi)))**(.5))
        if t > 5:
            pubMobile.publish(msg)
            msg = Float64()
            msg.data = upDownAngle*phase
            pubCam.publish(msg)
            msg.data = heaveLength*phase
            pubCamPris.publish(msg)
        else:
            msg = Float64()
            msg.data = 0 
            pubCam.publish(msg)
            pubCamPris.publish(msg)
        
        msg = Float64()
        msg.data = phase
        pubPhase.publish(msg)
        rate.sleep()
        t = t + 1.0/100.0

if __name__ == '__main__':

    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    

    try:
        spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        spawner('mobile_robot', open("/home/hans/catkin_ws/src/ros-mobile-robot-camera/mobile_robot_description/xacro/robot.xacro",'r').read(), "/mobile_robot", Pose(position= Point(0,0,.7),orientation=Quaternion(0,0,0,0)),"world")
    except rospy.ServiceException as e:
        print("Service call failed: ",e)

    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    

