import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
import matplotlib.animation as animation

data01 = np.loadtxt("./data01.txt")
#fig = plt.figure(figsize = (6,6))
#ax = fig.add_subplot(111)
#ax = plt.subplots()
fig, ax = plt.subplots()

ax.set_xlim(-0.5, 2.5)
ax.set_ylim(-0.5, 2.5)

grid_num = 3
resolution = 1

#ロボットの速度、角速度、
linear_x = 0
angular_z = 0
A = 0
B = 0
#信号受信間隔
t = 0.024

#ロボットの初期座標(0~grid_num)
x_r = 0
y_r = 0

#x,y座標の移動量a,b
a = 0
b = 0
theta = np.pi #* 1/2
#theta = 0

#0の行列での位置の取り出し
([x0,y0]) = np.where(data01 == 1 )
#配列の始まりと個数の始まりの1のズレを修正し、行列のy軸の向きからxy座標の向きに変換
y0 = grid_num-1-y0

#ロボットによる移動・回転
def callback(vel_msg):
    global linear_x,angular_z
    linear_x = vel_msg.linear.x
    angular_z = vel_msg.angular.z
def listener():
    rospy.init_node('robot_cleaner' , anonymous=True)

    rospy.Subscriber("/turtle1/cmd_vel",Twist,callback)

    #1回の信号での変位
    A = linear_x * t
    B = angular_z * t
    #環境の変位
    x_d = ((x0-x_r)*np.cos(B) + (y0-y_r)*np.sin(B) - A*np.cos(B))/resolution
    y_d = (-(x0-x_r)*np.sin(B) + (y0-y_r)*np.cos(B) - A*np.sin(B))/resolution
    print(x_d,y_d)

    #print(len(x0))
    #print(x0,y0)

    #affine transformation、アフィン変換を行う
    #x00 =  x0*np.cos(theta) - y0*np.sin(theta) + a
    #obs_mapではxy座標であり行列の配列とy軸の向きが反対であるため-bとする
    #y00 =  x0*np.sin(theta) + y0*np.cos(theta) - b

    #x00 = ((x_d-x_r)*np.cos(B) - (y_d-y_r)*np.sin(B) + A*np.cos(B))/resolution
    #y00 = ((x_d-x_r)*np.sin(B) + (y_d-y_r)*np.cos(B) + A*np.sin(B))/resolution

    #print(x00,y00)

    #下のforループで得たx1,y1からx,y座標の位置を整数型で算出しreturnで返す
    def real2grid_index_fixed_grid_num(x1, y1, resolution):
        #return np.floor(x1/resolution).astype(int),\
        #       np.floor(y1/resolution).astype(int)
        return round(x1/resolution),\
               round(y1/resolution)

    obs_map = np.zeros([grid_num, grid_num])

    #print(len(x00))
    print(len(x_d))

    #0の位置にobstacleの割当を行う、iは0が該当した個数分ループさせる
    #for i in range(len(x00)):
    #    x1 = x00[i]
    #    y1 = y00[i]
    for i in range(len(x_d)):
        x1 = x_d[i]
        y1 = y_d[i]
        print(real2grid_index_fixed_grid_num(x1, y1, resolution))
        print(x1,y1)
        print(x1,grid_num-y1)
        #座標の範囲内に
        if 0 <= x1 <= 2 and 0 <= y1 <= 2:
            x_idx, y_idx = real2grid_index_fixed_grid_num(x1, y1, resolution)
            print(x_idx,y_idx)
            obs_map[y_idx][x_idx] = 1
    #plt.imshow(obs_map)
    ax.imshow(obs_map)
    plt.pause(0.01)
    #ani = animation.FuncAnimation(fig, obs_map, interval = 20)

    rospy.spin()

if __name__=='__main__':
    listener()
