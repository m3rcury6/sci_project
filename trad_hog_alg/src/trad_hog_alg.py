#!/usr/bin/env python

'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: simple color detection for cone in order to start testing counter node
'''

#NN_NOTE: Init: clear this out
import rospy
import time
import cv2
from sys import argv
import numpy as np
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from skimage.feature import hog
from skimage import data, exposure
import common_tools.lib_tools as lib




# Gets HOG of image
# Method 1: Use skimage: NN_NOTE: Personally dont prefer
'''
fd, hog_image = hog(image, orientations=8, pixels_per_cell=(4, 4),
                    cells_per_block=(1, 1), visualize=True, multichannel=True)
'''
# Method 2: Use cv2 inbuilt HOG descriptor but cannot visualize
hog = cv2.HOGDescriptor()
hog_features = []

'''
Depending on input: needs a for loop that trains the hog or directly evaluates the incoming images:
res = cv2.resize(cone, (64, 128))
    h = hog.compute(res)
    #print np.shape(hog_features)
    hog_features.append(h)
'''

# Train SVM
# NN_NOTE: IS the node already supposed to be trained on a training set or split the incoming set?
samples = np.array(np.random.random((4,2)), dtype = np.float32)
y_train = np.array([1.0,0.0,0.0,1.0], dtype = np.int32)

clf = cv2.ml.SVM_create()
clf.setKernel(cv2.ml.SVM_LINEAR)
clf.setType(cv2.ml.SVM_C_SVC)
clf.setC(2.67)
clf.setGamma(5.383)

clf.train(samples, cv2.ml.ROW_SAMPLE, y_train)

y_val = clf.predict(samples)

# NN_NOTE: ROS: Still subscribing to /images and /image_name nodes
rospy.init_node('hog_algorithm_node')
rospy.Subscriber('images',Image,call_img,queue_size=10)
rospy.Subscriber('image_name',String,call_name,queue_size=10)
pub_result = rospy.Publisher('detector_result',String,queue_size=10)


while(not rospy.is_shutdown()):
	rospy.spin()
