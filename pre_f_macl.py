#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import rospy
import cv2
#from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import collections

data01 = np.loadtxt("./data02.txt")
fig, ax = plt.subplots()

grid_num = 10
resolution = 1

#ロボットの速度、角速度
linear_x = 0
angular_z = 0
#ロボットの位置、角度、過去角度
pose_x = 0
pose_y = 0
pose_theta = 0
theta = 0
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
    #return np.floor(x1/resolution).astype(int),np.floor(y1/resolution).astype(int)
    return np.floor(x1/resolution),np.floor(y1/resolution)
#def real2grid_index_fixed_grid_num(xy, resolution):
    #return np.floor(xy/resolution).astype(int)

def pose_callback(pose):
    global pose_x,pose_y,pose_theta
    pose_x = pose.x - 5.544444562
    pose_y = pose.y - 5.544444562
    pose_theta = pose.theta

def listener():
    rospy.init_node('robot_cleaner' , anonymous=True)
    rospy.Subscriber("/turtle1/pose",Pose,pose_callback)

if __name__=='__main__':
    listener()

q = 0
while not rospy.is_shutdown():
    dtheta = pose_theta - theta
    theta = theta + dtheta
    r = [np.sqrt(np.square(x0[e]-x_r) + np.square(y0[e]-y_r)) for e in range(len(x0))]
    #x_d = [x0[e] - pose_x for e in range(len(x0))]
    #y_d = [y0[e] - pose_y for e in range(len(x0))]
    x_d = [x0[e] - pose_x - r[e]*np.cos(dtheta - np.pi*1/2) for e in range(len(x0))]
    y_d = [y0[e] - pose_y - r[e]*np.sin(dtheta) for e in range(len(x0))]
    #mapの範囲内にある障害物のindexを調べる
    def func1(lst, value_a,value_b):
      return [h for h, x in enumerate(lst) if value_a <= x <= value_b]
    x_idx = func1(x_d,0,grid_num-1)
    y_idx = func1(y_d,0,grid_num-1)
    #x,y両方共mapの範囲内にある障害物のindexを調べる
    a = x_idx + y_idx
    def func2(l):
      return [k for k, v in collections.Counter(l).items() if v > 1]
    idx = func2(a)
    #該当する障害物のindexを表示
    #global x_n_idx,y_n_idx
    x_val = [x_d[j] for j in idx]
    y_val = [y_d[j] for j in idx]
    #print(type(x_n_idx))
    #print(x_val)

    obs_map = np.zeros([grid_num, grid_num])
    s = str(q) #数値を文字列に

    x_n_idx, y_n_idx = real2grid_index_fixed_grid_num(np.array(x_val), np.array(y_val), resolution)
    xy_idx = (x_n_idx+np.array(y_n_idx)*grid_num).astype(int)
    #print(xy_idx)
    np.put(obs_map,xy_idx,1,mode='raise')
    print(obs_map)
    ax.imshow(obs_map)
    #for i in range(len(x_n_idx)):
    #    x2 = x_n_idx[i]
    #    y2 = y_n_idx[i]
    #    x_k_idx, y_k_idx = real2grid_index_fixed_grid_num(x2, y2, resolution)
        #print(x_k_idx,type(x_k_idx))
    #    obs_map[y_k_idx][x_k_idx] = 1
    #    print(obs_map)
    #    ax.imshow(obs_map)
    plt.savefig("../../デスクトップ/test/image_obs_map/obs_map/obs_map_pfm_"+s)
    img = cv2.imread("../../デスクトップ/test/image_obs_map/obs_map/obs_map_pfm_"+s +".png")
    img1 = img[60:425, 145:510]
    cv2.imwrite("../../デスクトップ/test/image_obs_map/obs_map/obs_map_pfmc_"+s +".png",img1)

    sp = str(q-1)
    if q ==0:
        pass
    #1枚目(0枚目)のbgr画像を作成
    if q == 1:
        im_0 = cv2.imread("../../デスクトップ/test/image_obs_map/obs_map/obs_map_pfmc_1.png")
        # bgrでの色抽出
        bgrLower = np.array([36, 231, 253])
        bgrUpper = np.array([255, 255, 255])
        img_mask = cv2.inRange(im_0, bgrLower, bgrUpper) # bgrからマスクを作成
        im_00 = cv2.bitwise_and(im_0, im_0, mask=img_mask) # 元画像とマスクをAND演算で合成

        # 特定の色を別の色に置換する
        before_color = [36, 231, 253]
        after_color = [255, 0, 0]
        im_00[np.where((im_00 == before_color).all(axis=2))] = after_color
        im_af_l= cv2.line(im_00,(145,365),(145,0),(0,0, 255),3)
        cv2.imwrite('../../デスクトップ/test/image_obs_map/after_image/after_image_pfmc_1.png',im_af_l)

    #(擬似的な過去4枚分を)1/5ずつ輝度値を下げて最新のbgr画像を重ね合わせる
    if q > 1:
        im_p = cv2.imread("../../デスクトップ/test/image_obs_map/after_image/after_image_pfmc_" +sp +".png")
        # 一つ前の画像に対して輝度値を1/5ずつ下げる
        before_color_0 = [0,255,0]
        after_color_0 = [0, 0, 0]
        im_p[np.where((im_p == before_color_0).all(axis=2))] = after_color_0
        before_color_1 = [51,0,0]
        after_color_1 = [0, 0, 0]
        im_p[np.where((im_p == before_color_1).all(axis=2))] = after_color_1
        before_color_2 = [102,0,0]
        after_color_2 = [51, 0, 0]
        im_p[np.where((im_p == before_color_2).all(axis=2))] = after_color_2
        before_color_3 = [153,0,0]
        after_color_3 = [102, 0, 0]
        im_p[np.where((im_p == before_color_3).all(axis=2))] = after_color_3
        before_color_4 = [204, 0, 0]
        after_color_4 = [153, 0, 0]
        im_p[np.where((im_p == before_color_4).all(axis=2))] = after_color_4
        before_color_5 = [255, 0, 0]
        after_color_5 = [204, 0, 0]
        im_p[np.where((im_p == before_color_5).all(axis=2))] = after_color_5

        im_n = cv2.imread("../../デスクトップ/test/image_obs_map/obs_map/obs_map_pfmc_" +s +".png")
        #最新画像のbgr画像を作成
        bgrLower_0 = np.array([36, 231, 253])
        bgrUpper_0 = np.array([255, 255, 255])
        img_mask = cv2.inRange(im_n, bgrLower_0, bgrUpper_0)
        im_nest = cv2.bitwise_and(im_n, im_n, mask=img_mask)
        before_color_6 = [36, 231, 253]
        after_color_6 = [255, 0, 0]
        im_nest[np.where((im_nest == before_color_6).all(axis=2))] = after_color_6
        #輝度値を下げた過去画像と最新画像を(過去画像の色を残すため)OR演算で重ね合わせ
        im_af = cv2.bitwise_or(im_p,im_nest)
        im_af_l= cv2.line(im_af,(145,365),(145,0),(0,0, 255),3)
        if q%4 == 0:
            im_af_lml = cv2.line(im_af_l,(108,365),(108,0),(0,255,0),3)
        if q%4 == 1:
            im_af_lml= im_af_l
        if q%4 == 2:
            im_af_lml = cv2.line(im_af_l,(182,365),(182,0),(0,255,0),3)
        if q%4 == 3:
            im_af_lml= im_af_l
        cv2.imwrite('../../デスクトップ/test/image_obs_map/after_image/after_image_pfmc_' +s +'.png',im_af_lml)

    q = q + 1

    plt.pause(0.2)
