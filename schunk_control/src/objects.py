#!/usr/bin/env python
import roslib
import os
import os.path
import sys
import signal
import rospy
import rosparam

from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from vision_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from socialrobot_motion.srv import *
from socialrobot_hardware.srv import *
from socialrobot_behavior.srv import *

##############################
# Main function
##############################
if __name__ == '__main__':
    rospy.init_node('objects')

    plan_req = MotionPlanRequest()

    # obstacles 
    plan_req.obstacle_ids = ['obj_table', 'obj_juice','obj_milk']
    #plan_req.targetObject = 'obj_juice'
    # table
    obs = BoundingBox3D()
    c = Pose()
    c.position.x = 0.57500565052
    c.position.y = 0.0250088647008
    c.position.z = 0.36501121521
    c.orientation.x = -0.499993562698
    c.orientation.y = 0.500006496906
    c.orientation.z = -0.500006496906
    c.orientation.w = -0.499993562698
    obs.center = c
    v = Vector3()
    v.x = 0.730000257492
    v.y = 0.750000059605
    v.z = 1.20000016689
    obs.size = v
    plan_req.obstacles.append(obs)

    # juice
    obs1 = BoundingBox3D()
    c1 = Pose()
    c1.position.x = 0.390748232603
    c1.position.y = 0.217293173075
    c1.position.z = 0.801787078381
    c1.orientation.x = -3.72598707443e-09
    c1.orientation.y = -1.3020775441e-05
    c1.orientation.z = -2.45272501579e-17
    c1.orientation.w = 1.0
    obs1.center = c1
    v1 = Vector3()
    v1.x = 0.0450000055134
    v1.y = 0.0450000055134
    v1.z = 0.143729999661
    obs1.size = v1
    plan_req.obstacles.append(obs1)

    # milk
    obs2 = BoundingBox3D()
    c2 = Pose()
    c2.position.x = 0.438998311758
    c2.position.y = 0.29629996419
    c2.position.z = 0.808184683323  
    c2.orientation.x = -3.72905573087e-09
    c2.orientation.y = -1.30315002025e-05
    c2.orientation.z = 1.12574913855e-16         
    c2.orientation.w = 1.0              
    obs2.center = c2
    v2 = Vector3()
    v2.x = 0.0450000055134
    v2.y = 0.0450000055134
    v2.z = 0.156530082226
    obs2.size = v2
    plan_req.obstacles.append(obs2)

    
