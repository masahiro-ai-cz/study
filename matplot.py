import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist

data01 = np.loadtxt("./data01.txt")
fig = plt.figure(figsize = (6,6))
ax = fig.add_subplot(111)
#ax = plt.subplots()
#fig, ax = plt.subplots()

grid_num = 3
resolution = 1

#ロボットの初期座標(0~grid_num)
x_r = 1
y_r = 1
#ロボットによる移動・回転


#x,y座標の移動量a,b
a = 1
b = 1
#theta = np.pi * 1/4
theta = 0

#0の行列での位置の取り出し
([x0,y0]) = np.where(data01 == 0 )
#([x1,y1]) = np.where(data01 == 1)

#print(len(x0))
#print(x0,y0)

#affine transformation、アフィン変換を行う
x00 =  x0*np.cos(theta) - y0*np.sin(theta) + a
y00 =  x0*np.sin(theta) + y0*np.cos(theta) + b
#[x1] =  [x1*math.cos(theta) - y1*math.sin(theta) + a]
#[y1] =  [x1*math.sin(theta) + y1*math.cos(theta) + b]

print(x00,y00)

ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)

#下のforループで得たx1,y1からx,y座標の位置を整数型で算出しreturnで返す
def real2grid_index_fixed_grid_num(x1, y1, resolution):
    return np.floor(x1/resolution).astype(int),\
           np.floor(y1/resolution).astype(int)

obs_map = np.zeros([grid_num, grid_num])

print(len(x00))

#0の位置にobstacleの割当を行う、iは0が該当した個数分ループさせる
for i in range(len(x00)):
    x1 = x00[i]
    y1 = y00[i]
#    print(real2grid_index_fixed_grid_num(x1, y1, resolution))
    print(x1,y1)
    x_idx, y_idx = real2grid_index_fixed_grid_num(x1, y1, resolution)
#    print(x_idx,y_idx)
    obs_map[grid_num-y_idx][x_idx] = 1

#for x0,y0,x1,y1 in zip([x0],[y0],[x1],[y1]):
#        ax.scatter(x0+0.5, y0+0.5, s = 12136, marker = "s", color="black")
#        ax.scatter(x1+0.5, y1+0.5, s = 12136, marker = "s", color="blue")

plt.imshow(obs_map)
plt.show()
