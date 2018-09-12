#!/usr/bin/env python

'''
Objective: To publish images
Coder: Nitin Nayak
'''

import rospy
import time
from sys import argv
from std_msgs.msg import String

rospy.init_node('image_input')
pub_txt = rospy.Publisher('image_input',String,queue_size=10)

while(not rospy.is_shutdown()):
	pub_txt.publish('Publishing image'+str(time.time()))
	time.sleep(1)
