#!/usr/bin/env python

'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: simple color detection for cone in order to start testing counter node
'''

import rospy
import time
import cv2
from sys import argv
import numpy as np
import os

from std_msgs.msg import String


rospy.init_node('args_node')
pub_txt = rospy.Publisher('talk_node',String,queue_size=10)

while(not rospy.is_shutdown()):
	pub_txt.publish('hello world'+str(time.time()))
	time.sleep(1)
