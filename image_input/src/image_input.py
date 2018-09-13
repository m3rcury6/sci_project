#!/usr/bin/env python

'''
Objective: Publish images from folder
Authors: Nitin Nayak
'''

import rospy
import time
from sys import argv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#from PIL import Image
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
print 'currently located at : ',os.getcwd()

# 2.0: list images ######################
image_list = []
image = []
valid_images = [".jpg"]
for filename in os.listdir(read_path):
	ext = os.path.splitext(filename)[1]
	if ext.lower() not in valid_images:
		continue
	image_list.append(filename)

# 3.0: appends the images and displays them
for val in range(len(image_list)):
	image.append(cv2.imread(str(image_list[val]), 1))
	cv2.imshow(str(image_list[val]),image[val])
	cv2.waitKey(0)
	cv2.destroyAllWindows()


rospy.init_node('image_input')
pub_image = rospy.Publisher('images', Image, queue_size=10)
pub_image_name = rospy.Publisher('image_name', String, queue_size=10)


val = 0	#kills node once finished with all images
while(not rospy.is_shutdown() and val < len(image_list)):
	pub_image.publish(CvBridge().cv2_to_imgmsg(image[val], "bgr8"))
	pub_image_name.publish('Current image: ' + str(image_list[val]))
	time.sleep(1)
	val = val + 1
