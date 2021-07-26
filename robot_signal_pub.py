#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897
#information of forward
speed_m = 1
distance = 12
isForward = True #True or false
#information of rotate
angle = 270
#直進している間、目標の角度に回転させる
speed = angle * speed_m / distance
clockwise = True #True or false

def signal():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360
    #Checking if the movement is forward or backwards、全身は+、後退は-にする
    if(isForward):
        vel_msg.linear.x = abs(speed_m)
    else:
        vel_msg.linear.x = -abs(speed_m)
    # Checking if our movement is CW or CCW、Trueのとき時計回り、Falseのとき反時計回り
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    #Ctrl + cされない限りループ
    while not rospy.is_shutdown():

         #Setting the current time for distance,angle calculus、時間を取得し速度、角速度の計算に使用
         t0 = rospy.Time.now().to_sec()
         #初期の距離、初期角度を０に
         current_distance = 0
         current_angle = 0

         #Loop to move the turtle in an specified distance、目標角度までループ
         while(current_distance < distance):
             #Publish the velocity
             velocity_publisher.publish(vel_msg)
             #Takes actual time to velocity calculus
             t1=rospy.Time.now().to_sec()
             #Calculates distancePoseStamped、speedとt1、t0の時間差より距離の計算に
             current_distance= speed*(t1-t0)


         while(current_angle < relative_angle):
             velocity_publisher.publish(vel_msg)
             t1 = rospy.Time.now().to_sec()
             current_angle = angular_speed*(t1-t0)

         #Forcing our robot to stop、上記のループが終わったら速度、角速度の信号を0にして止める
         vel_msg.linear.x = 0
         vel_msg.angular.z = 0
         velocity_publisher.publish(vel_msg)
         rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        signal()
    except rospy.ROSInterruptException: pass
