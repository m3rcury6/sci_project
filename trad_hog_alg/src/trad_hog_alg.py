#!/usr/bin/env python

'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: Cone detection using Histogram of Gradients (HOG) with traditional machine learning method, Support Vector Method (SVM)
'''

## 1. INITIALIZATIONS
import rospy
import cv2
from sys import argv
import numpy as np
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String
from imutils.object_detection import non_max_suppression
import common_tools.lib_tools as lib                            # self-developed library of functions

'''
## 2. Move to testing folder                                    # not needed when test images are provided by ROS node
testing_path = argv[1]
if(not os.path.exists(testing_path)):
    print "Error, path below doesn't exist."
else:
    print 'Success: read training_path found.'
os.chdir(testing_path)
print 'Located at:',os.getcwd()
'''
## 3. Load .xml file of SVM classifier
if(not os.path.exists(testing_path)):
    print "Error, path below doesn't exist."
else:
    print 'Success: read training_path found.'
os.chdir(testing_path)
clf = cv2.ml.SVM_load('../HOG_svm_model.xml')
print 'Loaded file'

'''
## 4.  Read a test image, resize it, save a copy                # not needed when test images are provided by ROS node
img = cv2.imread('01_12.jpg')
img = cv2.resize(img, (0,0), fx=2, fy=2)
orig = img.copy()
'''

## 5. Extract support vectors from SVm
svm = clf.getSupportVectors()
svm = np.transpose(svm)

## 6. Set the SVm to a HOG descriptor
hog = cv2.HOGDescriptor()
hog.setSVMDetector(svm)

## 7. Detect cones in the image
(rects, weights) = hog.detectMultiScale(img, winStride=(16, 16), padding=(0, 0), scale=1.2, finalThreshold = 1)
print('LENGTH: ',len(rects))

'''
## 8. Draw the detected bounding boxes
for (x, y, w, h) in rects:
    cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
'''
## 9. Apply Non Max Suppression
rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
pick = non_max_suppression(rects, probs=None, overlapThresh=0.0)
'''
## 10. Draw NMS bounding boxes
for (xA, yA, xB, yB) in pick:
    cv2.rectangle(img, (xA, yA), (xB, yB), (255, 0, 0), 2)
'''
## 11. Resize images back to original sizes
#orig = cv2.resize(orig, (0,0), fx=0.5, fy=0.5)
img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)

'''
## 12. Show the images
cv2.imshow('Test Image',orig)
cv2.imshow("After NMS", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''

## Initialise ROS node and run it
rospy.init_node('hog_algorithm_node')
#pub_result = rospy.Publisher('detector_result',String,queue_size=10)
while(not rospy.is_shutdown()):
	rospy.spin()
