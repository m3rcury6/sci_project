#!/usr/bin/env python

'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: Cone detection using Histogram of Gradients (HOG) with traditional machine learning method, Support Vector Method (SVM)
'''

## 1. INITIALIZATIONS
import rospy
#import time
import cv2
from sys import argv
import numpy as np
import os
#from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

#from skimage.feature import hog
#from skimage import data, exposure
import common_tools.lib_tools as lib                            # self-developed library of functions
from imutils.object_detection import non_max_suppression        # used for NMS of rectangles after detection

## 2. READ PATH OF POSITIVE IMAGES
training_path_positive = argv[1]
if(not os.path.exists(training_path_positive)):
    print "Error, path below doesn't exist."
else:
    print 'Success: read training_path found.'
os.chdir(training_path_positive)
print 'Located at:',os.getcwd()





# Gets HOG of image
# Method 1: Use skimage: NN_NOTE: Personally dont prefer
'''
fd, hog_image = hog(image, orientations=8, pixels_per_cell=(4, 4),
                    cells_per_block=(1, 1), visualize=True, multichannel=True)
'''


# NN_NOTE: ROS: Still subscribing to /images and /image_name nodes
rospy.init_node('hog_algorithm_node')
#rospy.Subscriber('images',Image,call_img,queue_size=10)
#rospy.Subscriber('image_name',String,call_name,queue_size=10)
pub_result = rospy.Publisher('detector_result',String,queue_size=10)

while(not rospy.is_shutdown()):
	rospy.spin()
