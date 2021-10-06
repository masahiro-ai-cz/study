# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import rospy
#from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import matplotlib.animation as animation
import time
import collections

data01 = np.loadtxt("./data01.txt")
#fig = plt.figure(figsize = (6,6))
#ax = fig.add_subplot(111)
#ax = plt.subplots()
fig, ax = plt.subplots()

#ax.set_xlim(-0.5, 2.5)
#ax.set_ylim(-0.5, 2.5)

grid_num = 10
resolution = 1

#ロボットの速度、角速度、
linear_x = 0
angular_z = 0
pose_x = 0
pose_y = 0
A = 0
B = 0
#信号受信間隔
t = 0.024
#ロボットの位置、角度
pose_x = 0
pose_y = 0
pose_theta = 0

#ロボットの初期座標(0~grid_num)
x_r = 0 #-5.54
y_r = 0 #-5.54

#x,y座標の移動量a,b
a = 0
b = 0
theta = np.pi #* 1/2
#theta = 0

#0の行列での位置の取り出し
([x0,y0]) = np.where(data01 == 1 )
#配列の始まりと個数の始まりの1のズレを修正し、行列のy軸の向きからxy座標の向きに変換
#print(x0,y0)
y0 = grid_num-1-y0
#print(y0)

r = np.zeros([len(x0)])
x_d = np.zeros([len(x0)])#.astype(int)
y_d = np.zeros([len(y0)])#.astype(int)
#print(x_d)

x_n_idx = []
y_n_idx = []

#ロボットによる移動・回転
#def callback(vel_msg):
    #global linear_x,angular_z
#    linear_x = vel_msg.linear.x
#    angular_z = vel_msg.angular.z
    #while not rospy.is_shutdown():
    #if linear_x == 0 and angular_z == 0:
    #    continue
def real2grid_index_fixed_grid_num(x1, y1, resolution):
    return np.floor(x1/resolution).astype(int),np.floor(y1/resolution).astype(int)
    #return round(x1/resolution),\
    #       round(y1/resolution)
def pose_callback(pose):
    #1回の信号での変位
    #A = linear_x * t
    #B = angular_z * t
    #環境の変位
    #x_d = ((x0-x_r)*np.cos(B) + (y0-y_r)*np.sin(B) - A*np.cos(B))/resolution
    #y_d = (-(x0-x_r)*np.sin(B) + (y0-y_r)*np.cos(B) - A*np.sin(B))/resolution
    #ロボットの変位
    #global r , x_d , y_d
    pose_x = pose.x - 5.544444562
    pose_y = pose.y - 5.544444562
    pose_theta = pose.theta
    #print(pose_x,pose_y,pose_theta)
    #環境の変位
    #r = np.sqrt(np.square(x0-x_r) + np.square(y0-y_r))
    #global x_d, y_d
    pose_d_theta = pose.theta - pose_theta
    for e in range(len(x_d)):
        #環境の変位
        r[e] = np.sqrt(np.square(x0[e]-x_r) + np.square(y0[e]-y_r))
        x_d[e] = x0[e] - pose_x #- r[e]*np.cos(pose_d_theta)
        y_d[e] = y0[e] - pose_y #- r[e]*np.sin(pose_d_theta)
    #print(x_d,y_d)
    def func1(lst, value_a,value_b):
        return [q for q, x in enumerate(lst) if value_a <= x <= value_b]
    x_idx = func1(x_d,0,grid_num-1)
    #print(x_idx)
    y_idx = func1(y_d,0,grid_num-1)
    #print(y_idx)
    a = x_idx + y_idx
    #print(a)
    def func2(l):
        return [k for k, v in collections.Counter(l).items() if v > 1]
    idx = func2(a)
    #print(idx)
    global x_n_idx, y_n_idx
    for j in idx:
        #print(j)
        #print(x_d[j],y_d[j])
        x_n_idx.append(x_d[j])
        y_n_idx.append(y_d[j])
    #print(x_n_idx,y_n_idx)
    #print(x_d,y_d)
    #print(len(x0))
    #print(x0,y0)
    #return x_d,y_d

    #affine transformation、アフィン変換を行う
    #x00 =  x0*np.cos(theta) - y0*np.sin(theta) + a
    #obs_mapではxy座標であり行列の配列とy軸の向きが反対であるため-bとする
    #y00 =  x0*np.sin(theta) + y0*np.cos(theta) - b

    #x00 = ((x_d-x_r)*np.cos(B) - (y_d-y_r)*np.sin(B) + A*np.cos(B))/resolution
    #y00 = ((x_d-x_r)*np.sin(B) + (y_d-y_r)*np.cos(B) + A*np.sin(B))/resolution

    #print(x00,y00)

    #下のforループで得たx1,y1からx,y座標の位置を整数型で算出しreturnで返す
#def obstacle_map():
    #for j in range(6000):
    #obs_map = np.zeros([grid_num, grid_num])

    #print(len(x00))
    #print(len(x_d))

    #0の位置にobstacleの割当を行う、iは0が該当した個数分ループさせる
    #for i in range(len(x00)):

    #    x1 = x00[i]
    #    y1 = y00[i]print(x_d,y_d)
    #for i in range(len(x_d)):
        #x1 = x_d[i]
        #y1 = y_d[i]
        #print(real2grid_index_fixed_grid_num(x1, y1, resolution))
        #print(x1,y1)
        #print(x1,grid_num-y1)
        #座標の範囲内に
        #if 0 <= x1 <= 2 and 0 <= y1 <= 2:
            #x_idx, y_idx = real2grid_index_fixed_grid_num(x1, y1, resolution)
            #print(x_idx,y_idx)
            #obs_map[y_idx][x_idx] = 1
        #print(x1,y1)
    #return obs_map
    #plt.imshow(obs_map)
    #ax.imshow(obs_map)
    #plt.pause(0.1)
    #ani = animation.FuncAnimation(fig, obs_map, interval = 20)
#print(x_n_idx,y_n_idx)
def listener():
    rospy.init_node('robot_cleaner' , anonymous=True)

    #rospy.Subscriber("/turtle1/cmd_vel",Twist,callback)
    rospy.Subscriber("/turtle1/pose",Pose,pose_callback)

    #while True:
    obs_map = np.zeros([grid_num, grid_num])

    #print(len(x00))
    #print(len(x_d))

    #0の位置にobstacleの割当を行う、iは0が該当した個数分ループさせる
    #for i in range(len(x00)):

    #    x1 = x00[i]
    #    y1 = y00[i]print(x_d,y_d)
    count = 0
    while count <= 1:
        plt.pause(0.01)
        count += 1
    for i in range(len(x_n_idx)):
        #print(x_n_idx,y_n_idx)
        x2 = x_n_idx[i]
        y2 = y_n_idx[i]
        #print(x1,y1)
        #print(x1,grid_num-y1)
        #座標の範囲内に
        #if 0 <= x2 <= grid_num-1 and 0 <= y2 <= grid_num-1:
        x_k_idx, y_k_idx = real2grid_index_fixed_grid_num(x2, y2, resolution)
        print(x_k_idx,y_k_idx)
        obs_map[y_k_idx][x_k_idx] = 1
        #for j in range
        ax.imshow(obs_map)
        #print(x1,y1)
        #obs_map = obstacle_map()
        #return obs_map
        #plt.imshow(obs_map)
        #ax.imshow(obs_map)
        plt.pause(0.01)
        #count += 1

    #print(x_d,y_d)
    #print "x_d",x_d

    #rospy.Timer(rospy.Duration(0.1),callback)

    rospy.spin()
    #while not rospy.is_shutdown():
        #rospy.sleep(0.1)

if __name__=='__main__':
    listener()
