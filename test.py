#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist

linear_x = 0
angular_z = 0
A = 0
B = 0

def callback(vel_msg):
    rospy.loginfo("Liner:%f",vel_msg.linear.x)
    rospy.loginfo("angular:%f",vel_msg.angular.z)
    now = rospy.get_rostime()
    #loginfoで取得した時間を表示させる
    rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
    #関数外にlinearとangularの値を出すためにlinear_xとangular_zをglobal関数として定義する
    global linear_x , angular_z
    linear_x = vel_msg.linear.x
    angular_z = vel_msg.angular.z
    print(linear_x,angular_z)

    #信号受信間隔
#    t = 1
#    global A ,B
    #1回の信号での変位
#    A = linear_x * t
#    B = angular_z * t
#    print(A,B)

#信号受信間隔
#t = 1
#1回の信号での変位
#A = linear_x * t
#B = angular_z * t
    #環境の変位
#    x_d = ((x0-x_r)*np.cos(B) + (y0-y_r)*np.sin(B) - A*np.cos(B))/resolution
#    y_d = (-(x0-x_r)*np.sin(B) + (y0-y_r)*np.cos(B) - A*np.sin(B))/resolution

#    print(vel_msg.angular.z * 180/3.14159236)
#print(A,B)

def listener():
    rospy.init_node('robot_cleaner' , anonymous=True)

    rospy.Subscriber("/turtle1/cmd_vel",Twist,callback)

    rospy.spin()

if __name__=='__main__':
    listener()
