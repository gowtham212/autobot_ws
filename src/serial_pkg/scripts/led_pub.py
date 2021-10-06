#!/usr/bin/python3
import rospy
from std_msgs.msg import  Float64

from random import randint
from time import sleep


rospy.init_node('LED_NODE')
pub = rospy.Publisher('led_toggle', Float64, queue_size=10)

while not rospy.is_shutdown():
        rnd = randint(0,3)
        print("the number is " + str(rnd))
        message=Float64(rnd)
        pub.publish(message)
        rate = rospy.Rate(100)
        sleep(3)
rate.sleep()