#!/usr/bin/env python

'''
Objective: hello world style of ros and github initialization
'''

import rospy
import time
from sys import argv
from std_msgs.msg import String

rospy.init_node('args_node')
pub_txt = rospy.Publisher('talk_node',String,queue_size=10)

while(not rospy.is_shutdown()):
	pub_txt.publish('hello world'+str(time.time()))
	time.sleep(1)
