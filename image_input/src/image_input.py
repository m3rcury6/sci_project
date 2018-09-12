#!/usr/bin/env python

'''
Objective: Publish images from folder
Authors: Nitin Nayak
'''

import rospy
import time
from sys import argv
from std_msgs.msg import String
import os
import numpy as np
import cv2

# 1.0: Ensure read path is available ##########################################
read_path = argv[1]
print 'path:'
print read_path
if(not os.path.exists(read_path)):
	print "error, path below doesn't exist. creating..."
	print 'currently located at:',os.getcwd()
	#print read_path
	#os.mkdir(read_path)
os.chdir(read_path)
print 'currently located at:',os.getcwd()

# 2.0: display image ######################
img =  cv2.imread('00001.jpg', 1)
cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()

rospy.init_node('image_input')
pub_txt = rospy.Publisher('image_input',String,queue_size=10)

while(not rospy.is_shutdown()):
	pub_txt.publish('Publishing image'+str(time.time()))
	time.sleep(1)
