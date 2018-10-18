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
print 'A. Located at:',os.getcwd()

## 2.1 READ ALL POSITIVE IMAGE AND LABEL FILES, SORT THESE FILES, CREATE AN ARRAY OF ALL IMAGES AND LABELS
image_list = []         # array of names all valid images files
text_list = []          # array of names all valid label files
image = []              # array of opened image data (cv2 format)
matrix = []             # array of labelled data
valid_images = [".jpg"]
valid_txtfiles = [".txt"]
# Required arrays created
for filename in os.listdir(training_path_positive):
    ext = os.path.splitext(filename)[1]
    if ext.lower() in valid_images:
        image_list.append(filename)
    elif ext.lower() in valid_txtfiles:
        text_list.append(filename)
# You have list of image names and text files
# print 'Image names and Text files List created'
image_list.sort()
text_list.sort()
for val in range(len(image_list)):
    image.append(cv2.imread(str(image_list[val]), 1))
    matrix.append(np.loadtxt(str(text_list[val])))
# at this point, have list of image content (cv2 arrays) and image co-ordinates
print 'B. Positive Images and Cone Data obtained'

## 2.2 EXTRACT BLUE CONES FROM THE IMAGES USING THE LABEL FILES
# 2.2.1 Get the ROIs in the images
roi_val = []
for i in range(len(image)):
    size = np.shape(image[i])
    rh = size[0]
    cw = size[1]
    multiplier = np.asarray([1, cw, rh, cw, rh])    #class, xcP, ycP, wdP, htP (P = percentage)
    roi_val.append(matrix[i] * multiplier)       #xc,yc, wd, ht ;xcP*cw , ycP*rh, wdP*cw, htP*rh (cw=columnwidth, rh=rowheight, which are for image dimensions)

# 2.2.2 Get ROI co-ordinates in the images
boxes = []
for values in range(len(roi_val)):
    box = np.array([[],[],[],[]]).transpose()
    if roi_val[values].ndim == 1:
        roi_val[values] = roi_val[values].reshape((1,-1))
    for val in roi_val[values]:
        x1 = val[1] - val[3]/2.0
        x2 = val[1] + val[3]/2.0
        y1 = val[2] - val[4]/2.0
        y2 = val[2] + val[4]/2.0
        new = np.array([x1,y1,x2,y2])
        box = np.row_stack((box,new))
    box = box.astype(int)   # co-ordinates need to be in int for pixels
    boxes.append(box)       # array of co-ord arrays for each image
print 'C. ROI co-ordinates obtained'

## 3. COMPUTE HOG FOR POSITIVE IMAGES (chould it be converted to HSV or grayscale??)
hog = cv2.HOGDescriptor()
hog_features = []
for data in range(len(image)):
    for ibox in boxes[data]:
        image[data] = cv2.rectangle(image[data],tuple(ibox[:2]),tuple(ibox[2:]),(255,0,0))
        cone = image[data][ibox[1]:ibox[3], ibox[0]:ibox[2]]
        res = cv2.resize(cone, (64, 128))
        # CONVERT TO GRAYSCALE OR HSV HERE
        h = hog.compute(res)
        hog_features.append(h)
print 'D. HOG computed'
print np.shape(hog_features)


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
