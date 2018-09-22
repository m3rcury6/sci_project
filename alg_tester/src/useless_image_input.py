#!/usr/bin/env python

'''
Objective: Publish images from folder to a ROS topic
Author: Nitin Nayak
'''

import rospy
import time
from sys import argv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cv2

# # kjgnote: not using this section
# rospy.init_node('image_input')
# pub_image = rospy.Publisher('images', Image, queue_size=10)
# pub_image_name = rospy.Publisher('image_name', String, queue_size=10)

# 1.0: Ensure read path is available ##########################################
read_path = argv[1]
print 'path:'
print read_path
if(not os.path.exists(read_path)):
	print "error, path below doesn't exist. creating..."
	print 'currently located at:',os.getcwd()
	os.mkdir(read_path)
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
	'''
	TO VIEW EACH IMAGE remove this comment
	cv2.imshow(str(image_list[val]),image[val])
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	'''

val = 0	#kills ROS node once finished with all images
raw_input('Press ENTER to continue... ')
print 'publishing images...'
while(not rospy.is_shutdown() and val < len(image_list)):
	print 'publishing:',str(image_list[val])
	pub_image_name.publish(str(image_list[val]))
	pub_image.publish(CvBridge().cv2_to_imgmsg(image[val], "bgr8"))
	time.sleep(0.1)
	val += 1
