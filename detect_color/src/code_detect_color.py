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
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

def select_rgb_mask(image,rgbthresh=[128,32,2,210,81,9]):

    lower = np.uint8(rgbthresh[:3])    # Lower threshold (BLUE, GREEN, RED)
    upper = np.uint8(rgbthresh[3:])      # Upper threshold (BLUE, GREEN, RED)
    masked = cv2.inRange(image, lower, upper)    # Threshold the image, all in the threshold is white, all other black
    return masked

def qs(img):
    import cv2
    cv2.imshow('CLOSE WITH KEYBOARD',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

img_name = ''
def call_name(data):
	global img_name
	img_name = data.data

def call_img(data):
	print "=",time.ctime(time.time()),"========="
	print 'img',img_name,'loaded. detecting contours...'
	img = CvBridge().imgmsg_to_cv2(data,"bgr8")
	(_,all_contours,__) = cv2.findContours(select_rgb_mask(img),1,2)
	print 'found:',len(all_contours)
	pubstr = img_name+','+str(len(all_contours))
	print pubstr
	pub_result.publish(pubstr)


rospy.init_node('detector_color')
rospy.Subscriber('images',Image,call_img,queue_size=10)
rospy.Subscriber('image_name',String,call_name,queue_size=10)
pub_result = rospy.Publisher('detector_result',String,queue_size=10)

print 'waiting for data...'
while(not rospy.is_shutdown()):
	# # temporary stuff to check connectivity with counter
	# itemp=itemp+1
	# time.sleep(1)
	# pubstr='img_'+str(itemp)+','+str(itemp*2)
	# print pubstr
	# pub_result.publish(pubstr)
	# # temp stuff
	rospy.spin() # this is proper thing to do. will only publish when new image is processed.
