#!/usr/bin/env python
'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: simple listener for results from an attempt at detecting cones.


'''

import rospy
import time
from sys import argv
from std_msgs.msg import String
from std_msgs.msg import UInt8

def call_inc(data):
	print "i heard something",time.time()

rospy.init_node('count_node')
# pub_txt = rospy.Publisher('talk_node',String,queue_size=10)
rospy.Subscriber('count_topic',UInt8,call_inc,queue_size=10)
while(not rospy.is_shutdown()):
	rospy.spin()
