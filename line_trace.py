#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

Ko = 3.3
Kt = 3.5
Ke = 2#0.002
dt = 0.1

l_y = 3
theta = 0
eta = 0
angular_z = 0

def pose_callback(pose):
    global theta,eta
    theta = pose.theta
    eta = pose.y - l_y
    #print(type(eta))

def twist_callback(data):
    global angular_z
    angular_z = data.angular.z

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robot_cleaner', anonymous=True)
    rospy.Subscriber("turtle1/cmd_vel", Twist, twist_callback)
    rospy.Subscriber("turtle1/pose", Pose, pose_callback)

    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    #vel_msg = Twist()
    #vel_msg.linear.x = 0.5
    #vel_msg.linear.y = 0
    #vel_msg.linear.z = 0
    #vel_msg.angular.x = 0
    #vel_msg.angular.y = 0
    #vel_msg.angular.z = eta#angular_z - (Ko*angular_z + Kt*theta + Ke*eta) * dt
    #print(eta)

    rate = rospy.Rate(10)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_z - (Ko*angular_z + Kt*theta + Ke*eta) * dt
        pub.publish(vel_msg)
        print(vel_msg.angular.z)
        rate.sleep()

if __name__ == '__main__':
    listener()
