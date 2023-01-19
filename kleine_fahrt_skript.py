#!/usr/bin/env python

from __future__ import print_function

import math
import numpy
import sys
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Empty

def __init__():
    self.left_pub = None
    self.right_pub = None
    self.left_msg =None
    self.right_msg =None


def fahren():
    try:
    # Publisher
        left_pub = rospy.Publisher("/wamv/thrusters/left_thrust_cmd",Float32,queue_size=10)
        right_pub = rospy.Publisher("/wamv/thrusters/right_thrust_cmd",Float32,queue_size=10)
        rospy.init_node('kleineFahrt', anonymous=True) 
        left_msg = Float32()
        right_msg = Float32()
        rate = rospy.Rate(1) # 10 hz
        left_msg=1.5
        right_msg=0.5
        for i in range (0,4):
            left_pub.publish(left_msg)
            right_pub.publish(right_msg)
            rate.sleep()
        left_msg=1.0
        right_msg=1.0
        for i in range (0,10):
            left_pub.publish(left_msg)
            right_pub.publish(right_msg)
            rate.sleep()
        left_msg=0.6
        right_msg=1.5
        for i in range (0,3):
            left_pub.publish(left_msg)
            right_pub.publish(right_msg)
            rate.sleep()
        left_msg=0.8
        right_msg=1.0
        left_pub.publish(left_msg)
        right_pub.publish(right_msg)
        rate.sleep()
        left_msg=1.0
        right_msg=1.0
        for i in range (0,39):
            left_pub.publish(left_msg)
            right_pub.publish(right_msg)
            rate.sleep()
        left_msg=0.1
        right_msg=1.0
        for i in range (0,4):
            left_pub.publish(left_msg)
            right_pub.publish(right_msg)
            rate.sleep()
        left_msg=1.0
        right_msg=1.0
        for i in range (0,2):
            left_pub.publish(left_msg)
            right_pub.publish(right_msg)
            rate.sleep()
        left_msg=1.0
        right_msg=0.8
        left_pub.publish(left_msg)
        right_pub.publish(right_msg)
        rate.sleep()
        for i in range (0,5):
            left_pub.publish(left_msg)
            right_pub.publish(right_msg)
            rate.sleep()

	  
    except rospy.ServiceException as e:
        print("Fehler")


if __name__ == "__main__":   
    fahren ()