#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import rospy
#from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

pose_x = 0
dx = 0
x = 0
#pose_theta = 0
#dtheta = 0
#theta = 0

def pose_callback(pose):
    global pose_theta, pose_x
    pose_theta = pose.theta
    pose_x = pose.x

def listener():
    rospy.init_node('robot_cleaner' , anonymous=True)
    rospy.Subscriber("/turtle1/pose",Pose,pose_callback)

if __name__=='__main__':
    listener()

while not rospy.is_shutdown():
    print(pose_x)
    dx = pose_x - x
    print(dx)
    x = pose_x
    time.sleep(0.1)
    #a = {"pose_theta" : pose_theta}
    #print(a)
    #dtheta = pose_theta - theta
    #b = {"dtheta" : dtheta}
    #print(b)
    #theta = theta + dtheta
    #c = {"theta" : theta}
    #print(c)
