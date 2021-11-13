#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import rospy
import cv2
#from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import collections

data01 = np.loadtxt("./data02.txt")
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
  #mapの範囲内にある障害物のindexを調べる
  def func1(lst, value_a,value_b):
    return [q for q, x in enumerate(lst) if value_a <= x <= value_b]
  x_idx = func1(x_d,0,grid_num-1)
  y_idx = func1(y_d,0,grid_num-1)
  #x,y両方共mapの範囲内にある障害物のindexを調べる
  a = x_idx + y_idx
  def func2(l):
    return [k for k, v in collections.Counter(l).items() if v > 1]
  idx = func2(a)
  #該当する障害物のindexを表示
  global x_n_idx, y_n_idx
  x_n_idx = [x_d[j] for j in idx]
  y_n_idx = [y_d[j] for j in idx]
  #print(type(x_n_idx))

def listener():
    rospy.init_node('robot_cleaner' , anonymous=True)

    rospy.Subscriber("/turtle1/cmd_vel",Twist,twist_callback)
    rospy.Subscriber("/turtle1/pose",Pose,pose_callback)

if __name__=='__main__':
    listener()

k = 0
while not rospy.is_shutdown():
    obs_map = np.zeros([grid_num, grid_num])

    for i in range(len(x_n_idx)):
      x2 = x_n_idx[i]
      y2 = y_n_idx[i]
      x_k_idx, y_k_idx = real2grid_index_fixed_grid_num(x2, y2, resolution)
      #print(x_k_idx,type(x_k_idx))
      obs_map[y_k_idx][x_k_idx] = 1
      ax.imshow(obs_map)
      r = str(k) #数値を文字列に
      plt.savefig("../../デスクトップ/test/image_obs_map/obs_map/obs_map_pfm_"+r)

      rp = str(k-1)
      if k ==0:
          pass
      #1枚目(0枚目)のbgr画像を作成
      if k == 1:
          im_0 = cv2.imread("../../デスクトップ/test/image_obs_map/obs_map/obs_map_pfm_1.png")
          # bgrでの色抽出
          bgrLower = np.array([36, 231, 253])
          bgrUpper = np.array([255, 255, 255])
          img_mask = cv2.inRange(im_0, bgrLower, bgrUpper) # bgrからマスクを作成
          im_00 = cv2.bitwise_and(im_0, im_0, mask=img_mask) # 元画像とマスクをAND演算で合成

          # 特定の色を別の色に置換する
          before_color = [36, 231, 253]
          after_color = [255, 0, 0]
          im_00[np.where((im_00 == before_color).all(axis=2))] = after_color
          cv2.imwrite('../../デスクトップ/test/image_obs_map/after_image/after_image_pfm_1.png',im_00)
      #(擬似的な過去4枚分を)1/5ずつ輝度値を下げて最新のbgr画像を重ね合わせる
      if k > 1:
          im_p = cv2.imread("../../デスクトップ/test/image_obs_map/after_image/after_image_pfm_" +rp +".png")
          # 一つ前の画像に対して輝度値を1/5ずつ下げる
          before_color_0 = [51,0,0]
          after_color_0 = [0, 0, 0]
          im_p[np.where((im_p == before_color_0).all(axis=2))] = after_color_0
          before_color_1 = [102,0,0]
          after_color_1 = [51, 0, 0]
          im_p[np.where((im_p == before_color_1).all(axis=2))] = after_color_1
          before_color_2 = [153,0,0]
          after_color_2 = [102, 0, 0]
          im_p[np.where((im_p == before_color_2).all(axis=2))] = after_color_2
          before_color_3 = [204, 0, 0]
          after_color_3 = [153, 0, 0]
          im_p[np.where((im_p == before_color_3).all(axis=2))] = after_color_3
          before_color_4 = [255, 0, 0]
          after_color_4 = [204, 0, 0]
          im_p[np.where((im_p == before_color_4).all(axis=2))] = after_color_4

          im_n = cv2.imread("../../デスクトップ/test/image_obs_map/obs_map/obs_map_pfm_" +r +".png")
          #最新画像のbgr画像を作成
          bgrLower_0 = np.array([36, 231, 253])
          bgrUpper_0 = np.array([255, 255, 255])
          img_mask = cv2.inRange(im_n, bgrLower_0, bgrUpper_0)
          im_nest = cv2.bitwise_and(im_n, im_n, mask=img_mask)
          before_color_5 = [36, 231, 253]
          after_color_5 = [255, 0, 0]
          im_nest[np.where((im_nest == before_color_5).all(axis=2))] = after_color_5
          #輝度値を下げた過去画像と最新画像を(過去画像の色を残すため)OR演算で重ね合わせ
          im_af = cv2.bitwise_or(im_p,im_nest)
          #im_af_l= cv2.line(im_af,(0,0),(grid_num,grid_num),(0,0, 255),3)
          cv2.imwrite('../../デスクトップ/test/image_obs_map/after_image/after_image_pfm_' +r +'.png',im_af)

    k = k + 1

    plt.pause(0.01)
