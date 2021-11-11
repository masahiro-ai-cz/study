#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import rospy
#from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import matplotlib.animation as animation
import time
import collections

data01 = np.loadtxt("./data01.txt")
fig, ax = plt.subplots()

grid_num = 10
resolution = 1

#ロボットの速度、角速度
linear_x = 0
angular_z = 0
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
theta = np.pi

x_n_idx = []
y_n_idx = []

#行列での0の位置の取り出し
([x0,y0]) = np.where(data01 == 1 )
#配列の始まりと個数の始まりの1のズレを修正し、行列のy軸の向きからxy座標の向きに変換
#print(x0,y0)
y0 = grid_num-1-y0

def real2grid_index_fixed_grid_num(x1, y1, resolution):
    return np.floor(x1/resolution).astype(int),np.floor(y1/resolution).astype(int)

def twist_callback(vel_msg):
    global angular_z
    angular_z = vel_msg.angular.z

def pose_callback(pose):
  pose_x = pose.x - 5.544444562
  pose_y = pose.y - 5.544444562
  r = [np.sqrt(np.square(x0[e]-x_r) + np.square(y0[e]-y_r)) for e in range(len(x0))]
  x_d = [x0[e] - pose_x for e in range(len(x0))]
  y_d = [y0[e] - pose_y for e in range(len(x0))]
  def func1(lst, value_a,value_b):
    return [q for q, x in enumerate(lst) if value_a <= x <= value_b]
  x_idx = func1(x_d,0,grid_num-1)
  y_idx = func1(y_d,0,grid_num-1)
  a = x_idx + y_idx
  def func2(l):
    return [k for k, v in collections.Counter(l).items() if v > 1]
  idx = func2(a)
  global x_n_idx, y_n_idx
  x_n_idx = [x_d[j] for j in idx]
  y_n_idx = [y_d[j] for j in idx]

def listener():
    rospy.init_node('robot_cleaner' , anonymous=True)

    rospy.Subscriber("/turtle1/cmd_vel",Twist,twist_callback)
    rospy.Subscriber("/turtle1/pose",Pose,pose_callback)

if __name__=='__main__':
    listener()

while not rospy.is_shutdown():
    obs_map = np.zeros([grid_num, grid_num])

    for i in range(len(x_n_idx)):
      x2 = x_n_idx[i]
      y2 = y_n_idx[i]
      x_k_idx, y_k_idx = real2grid_index_fixed_grid_num(x2, y2, resolution)
      obs_map[y_k_idx][x_k_idx] = 1
      ax.imshow(obs_map)
      plt.pause(0.01)
