#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import rospy
import cv2
#from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import collections

x = 0
y = 0

resolution = 7/80
pixel = 365
grid_num = 80
#print(1/resolution * pixel/grid_num)

dx = x * 1/resolution * pixel/grid_num
dy = y * 1/resolution * pixel/grid_num

img = cv2.imread("image_obs_map/after_image/after_image_pfm_3.png")
img1 = img[60:425, 145:510]
cv2.imwrite("image_obs_map/cut_obs_map.png", img1)

imm = img1[0:315, 50:365]
cv2.imwrite("image_obs_map/cut_s_obs_map.png", imm)

#imb = np.zeros((365,365,3))
#cv2.imwrite('image_obs_map/after_image/black.png',imb)

img1 = cv2.imread('image_obs_map/after_image/black.png')
img2 = cv2.imread('image_obs_map/after_image/after_image_pfmc_9')
img3 = img[60:425, 145:510]

height, width = img3.shape[:2]
img1[50:height+50, 0:width] = img2

cv2.imwrite('image_obs_map/new.png', img1)
