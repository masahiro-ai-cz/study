#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897
#information of forward
speed_m = 1
distance = 3
isForward = True #True or false
#information of rotate
angle = 90
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
    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = abs(speed_m)
    else:
        vel_msg.linear.x = -abs(speed_m)
    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    while not rospy.is_shutdown():

         #Setting the current time for distance,angle calculus
         t0 = rospy.Time.now().to_sec()
         current_distance = 0
         current_angle = 0

         #Loop to move the turtle in an specified distance
         while(current_distance < distance):
             #Publish the velocity
             velocity_publisher.publish(vel_msg)
             #Takes actual time to velocity calculus
             t1=rospy.Time.now().to_sec()
             #Calculates distancePoseStamped
             current_distance= speed*(t1-t0)

         while(current_angle < relative_angle):
             velocity_publisher.publish(vel_msg)
             t1 = rospy.Time.now().to_sec()
             current_angle = angular_speed*(t1-t0)

         #Forcing our robot to stop
         vel_msg.linear.x = 0
         vel_msg.angular.z = 0
         velocity_publisher.publish(vel_msg)
         rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        signal()
    except rospy.ROSInterruptException: pass
