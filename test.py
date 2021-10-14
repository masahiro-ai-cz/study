#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist

linear_x = 0
angular_z = 0
A = 0
B = 0
resolution = 1
#信号受信間隔
t = 0.024
#ロボットの初期座標(0~grid_num)
x_r = 1
y_r = 1

#仮地図の値
x0 = 0
y0 = 0

def callback(vel_msg):
    #rospy.loginfo("Liner:%f",vel_msg.linear.x)
    #rospy.loginfo("angular:%f",vel_msg.angular.z)
    #now = rospy.get_rostime()
    #loginfoで取得した時間を表示させる
    #rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
    #関数外にlinearとangularの値を出すためにlinear_xとangular_zをglobal関数として定義する
    global linear_x , angular_z
    linear_x = vel_msg.linear.x
    angular_z = vel_msg.angular.z
#    print(linear_x,angular_z)
#    return linear_x,angular_z
#print(linear_x,angular_z)

#linear_x = v_msg(linear.x)
#angular_z = v_msg(angular.z)
#    print(linear_x,angular_z)

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
#    print(x_d,y_d)

#    print(vel_msg.angular.z * 180/3.14159236)
#print(A,B)

def listener():
    rospy.init_node('robot_cleaner' , anonymous=True)

    rospy.Subscriber("/turtle1/cmd_vel",Twist,callback)

    while not rospy.is_shutdown():
        print(linear_x,angular_z)

    #rospy.spin()

if __name__=='__main__':
    listener()

#a = np.loadtxt("./data01.txt")
#if a == 0:
#    x =
#    y =
#print(x)


#x=[1,2,3,1,2,3,1,2,3]
#y=[1,2,3,1,2,3,1,2,3]

#for i in range(len(x)):
#    x1=x[i]
#    y1=y[i]
#    print(x1,y1)

#x = [12,3,24,45,56]
#y = [56,7,33,78,32]
#print(x,y)
#for k in range(5):
#    print(x[k],y[k])

#for i in range (grid_num * grid_num):
#    (x0(i),y0(i)) = np.where(a == 0)
#    print(x0(i),y0(i))
#for x0 in
#    print(x0,y0)

#for x in range(9):
#    np.where(a == 1)
#    print(x)

# FigureとAxesを作成
#fig = plt.figure(figsize = (6, 6))
#ax = fig.add_subplot(111)

# 軸範囲を設定
#ax.set_xlim(0, 10)
#ax.set_ylim(0, 10)

# 3≦x≦6の範囲を塗り潰す
#ax.axvspan(0, 0, color = "coral")

#plt.show()

#([x1,y1]) = np.where(data01 == 1)
#[x1] =  [x1*math.cos(theta) - y1*math.sin(theta) + a]
#[y1] =  [x1*math.sin(theta) + y1*math.cos(theta) + b]
#for x0,y0,x1,y1 in zip([x0],[y0],[x1],[y1]):
#        ax.scatter(x0+0.5, y0+0.5, s = 12136, marker = "s", color="black")
#        ax.scatter(x1+0.5, y1+0.5, s = 12136, marker = "s", color="blue")
