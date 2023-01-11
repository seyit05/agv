#! /usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage 
from sensor_msgs.msg import Imu
import numpy as np
from std_msgs.msg import Float64 
from geometry_msgs.msg import Vector3 


global t
t = Imu()


def callback(msg):
    
    t.header.frame_id = msg.transforms[0].header.frame_id
    t.orientation.x = msg.transforms[0].transform.rotation.x
    t.orientation.y = msg.transforms[0].transform.rotation.y
    t.orientation.z = msg.transforms[0].transform.rotation.z
    t.orientation.w = msg.transforms[0].transform.rotation.w
    pub = rospy.Publisher('/imu_data',Imu, queue_size=1000)
    rate=rospy.Rate(10)
    pub.publish(t)

def callback2(msg2):
    
    t.linear_acceleration.x = msg2.x
    t.linear_acceleration.y = msg2.y
    t.linear_acceleration.z = msg2.z

    pub = rospy.Publisher('/imu_data',Imu, queue_size=1000)
    rate=rospy.Rate(10)
    pub.publish(t)

def callback3(msg3):

    t.angular_velocity.x = msg3.x
    t.angular_velocity.y = msg3.y
    t.angular_velocity.z = msg3.z
    pub = rospy.Publisher('/imu_data',Imu, queue_size=1000)
    rate=rospy.Rate(10)
    pub.publish(t)


if __name__ == '__main__':
    rospy.init_node('imu_message')
    rate = rospy.Rate(100) # 10hz
    sub = rospy.Subscriber('/tf', TFMessage, callback)
    sub2 = rospy.Subscriber('/accel', Vector3, callback2)
    sub3 = rospy.Subscriber('/gyro', Vector3, callback3)
    rospy.spin()