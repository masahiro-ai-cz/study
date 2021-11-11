#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from turtlesim.msg import Pose
import rospy
import cv2

fig, ax = plt.subplots()
grid_num = 10
pose_x = 0
pose_y = 0
#obs_map = np.zeros([grid_num, grid_num])

def pose_callback(pose):
    global pose_x,pose_y
    pose_x = np.floor(-pose.x + 5.544444562).astype(int)#-1,np.floor(pose.x - 5.544444561).astype(int),np.floor(pose.x - 5.544444561).astype(int)+1,np.floor(pose.x - 5.544444561).astype(int)-1,np.floor(pose.x - 5.544444561).astype(int)+1,np.floor(pose.x - 5.544444561).astype(int)-1,np.floor(pose.x - 5.544444561).astype(int),np.floor(pose.x - 5.544444561).astype(int)+1]
    pose_y = np.floor(-pose.y + 5.544444562).astype(int) + 5#+ 4,pose.y - 5.544444561).astype(int) + 4,pose.y - 5.544444561).astype(int) + 4,pose.y - 5.544444561).astype(int) + 5,pose.y - 5.544444561).astype(int) + 5,pose.y - 5.544444561).astype(int) + 6,pose.y - 5.544444561).astype(int) + 6,pose.y - 5.544444561).astype(int) + 6]
    #print(pose_x,pose_y)

def listener():
    rospy.init_node('robot_cleaner' , anonymous=True)
    rospy.Subscriber("/turtle1/pose",Pose,pose_callback)
    #rospy.spin()

if __name__ == '__main__':
    listener()
i = 0
while not rospy.is_shutdown():
    #print(pose_x,pose_y)
    obs_map = np.zeros([grid_num, grid_num])
    obs_map[pose_y][pose_x] = 1
    ax.imshow(obs_map)
    r = str(i)
    plt.savefig("/../../デスクトップ/test/image_obs_map/obs_map/obs_map_"+r)

    rp = str(i-1)
    #1枚目(0枚目)のbgr画像を作成
    if i == 0:
        im_0 = cv2.imread("/../../デスクトップ/test/image_obs_map/obs_map/obs_map_0.png")
        # bgrでの色抽出
        bgrLower = np.array([36, 231, 253])
        bgrUpper = np.array([255, 255, 255])
        img_mask = cv2.inRange(im_0, bgrLower, bgrUpper) # bgrからマスクを作成
        im_00 = cv2.bitwise_and(im_0, im_0, mask=img_mask) # 元画像とマスクをAND演算で合成

        # 特定の色を別の色に置換する
        before_color = [36, 231, 253]
        after_color = [255, 0, 0]
        im_00[np.where((im_00 == before_color).all(axis=2))] = after_color
        cv2.imwrite('/../../デスクトップ/test/image_obs_map/after_image/after_image_0.png',im_00)
    #(擬似的な過去4枚分を)1/5ずつ輝度値を下げて最新のbgr画像を重ね合わせる
    if i > 0:
        im_p = cv2.imread("/../../デスクトップ/test/image_obs_map/after_image/after_image_" +rp +".png")
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

        im_n = cv2.imread("/../../デスクトップ/test/image_obs_map/obs_map/obs_map_" +r +".png")
        #最新画像のbgr画像を作成
        bgrLower_0 = np.array([36, 231, 253])
        bgrUpper_0 = np.array([255, 255, 255])
        img_mask = cv2.inRange(im_n, bgrLower, bgrUpper)
        im_nest = cv2.bitwise_and(im_n, im_n, mask=img_mask)
        before_color_5 = [36, 231, 253]
        after_color_5 = [255, 0, 0]
        im_nest[np.where((im_nest == before_color_5).all(axis=2))] = after_color_5
        #輝度値を下げた過去画像と最新画像を(過去画像の色を残すため)OR演算で重ね合わせ
        im_af = cv2.bitwise_or(im_p,im_nest)
        im_af_l= cv2.line(im_af,(0,0),(grid_num,grid_num),(0,0, 255),3)
        cv2.imwrite('/../../デスクトップ/test/image_obs_map/after_image/after_image_' +r +'.png',im_af_l)

    i = i + 1
    plt.pause(1)
