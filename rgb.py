#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2

im = cv2.imread("/home/mouse/catkin_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/nodes/image/tbcnn_0.png")
#"image_obs_map/obs_map_27.png")
print(im)

#im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
#cv2.imwrite('image_obs_map/cv2_gray_obs_map_27.png', im_gray)

# bgrでの色抽出
bgrLower = np.array([36, 231, 253])    # 抽出する色の下限(bgr)
bgrUpper = np.array([255, 255, 255])    # 抽出する色の上限(bgr)
img_mask = cv2.inRange(im, bgrLower, bgrUpper) # bgrからマスクを作成
extract = cv2.bitwise_and(im, im, mask=img_mask) # 元画像とマスクを合成
#cv2.imwrite('image_obs_map/extract_1.png',extract)

# 特定の色を別の色に置換する
before_color = [36, 231, 253]
after_color = [255, 0, 0]
extract[np.where((extract == before_color).all(axis=2))] = after_color
#cv2.imwrite('image_obs_map/replace.png',extract)

grid_num = 10
obs_map = np.zeros([grid_num, grid_num])
obs_map = obs_map + 1
print(obs_map)

#cv2.imshow("imamge",extract)
#cv2.waitKey(0)
imc=extract[60:310, 202:452]
cv2.imwrite("/home/mouse/catkin_ws/src/turtlebot3_machine_learning/turtlebot3_dqn/nodes/image/raw_data/cut_s_tbcnn_0.png",imc)
