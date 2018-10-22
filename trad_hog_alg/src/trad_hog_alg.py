#!/usr/bin/env python

'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: Cone detection using Histogram of Gradients (HOG) with traditional machine learning method, Support Vector Method (SVM)
NOTE: DO NOT REMOVE COMMENTED OUT CODE
'''

## 1. INITIALIZATIONS
import rospy
import cv2
from sys import argv
import numpy as np
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from imutils.object_detection import non_max_suppression
import common_tools.lib_tools as lb
from ai_darknet.msg import bbox
from ai_darknet.msg import bbox_array
import time

## 2. Load .xml file of SVM classifier
path = argv[1]
if(not os.path.exists(path)):
    print "Error, path below doesn't exist."
else:
    print 'Success: path found.'
os.chdir(path)
clf = cv2.ml.SVM_load('../HOG_svm_model.xml')
print 'SVM Classifier .xml file successfully loaded'

## 3. Extract support vectors from SVM and set them to HOG descriptor
svm = clf.getSupportVectors()
svm = np.transpose(svm)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(svm)
print "SVM ready!!!"


## 4. The ROS loop thing
imgname = ""

def image_callback(data):
    global dnet
    img = CvBridge().imgmsg_to_cv2(data,"bgr8")                                                                         # reconvert to cv2
    img = cv2.resize(img, (0,0), fx=2, fy=2)                                                                            # resize for better detection
    orig = img.copy()                                                                                                   #save copy - MAY BE REMOVED
    (rects, weights) = hog.detectMultiScale(img, winStride=(16, 16), padding=(0, 0), scale=1.2, finalThreshold = 1)     # detect the cones
    num_of_detections = len(rects)                                                                                      # number of cones detected (very high false positives)
    print num_of_detections                                                                                             # MAY BE REMOVED
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])                                                  # prepare for NMS
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.0)                                                    # perform NMS
    #pick = cv2.resize(pick, (0,0), fx=0.5, fy=0.5)                                                                     # resize back to original size
    dnet = lb.pixx2dnet(pick, np.shape(pick))                                                                           # convert pixels to dnet format
    publish_bbox(dnet)                                                                                                  # publish dnet as bbox

def image_name_callback(data):
    global imgname
    imgname = data
    print 'publishing:',imgname

def publish_bbox(para):
    msgs = bbox_array() # generate new, empty custom message array
    for irow in para:
        imsg = bbox() # generate new, empty custom message for each bbox
        imsg.x = irow[0]
        imsg.y = irow[1]
        imsg.w = irow[2]
        imsg.h = irow[3]
        msgs.bboxes.append(imsg)
        # don't need to touch the other items (Class, prob)
    # forloop conversion
    msgs.header.frame_id = imgname # IMPORTANT: TRANSMIT IMAGE NAME HERE
    pub_bboxes.publish(msgs) # when ready, publish custom message array
    print "published!!"

## Initialise ROS node and run it
rospy.init_node('hog_algorithm_node')
print "*** NODE INTIIALISED ***"
print "RUN ALG_TESTER NODE"
rospy.Subscriber('images', Image, image_callback, queue_size=10)
rospy.Subscriber('image_name', String, image_name_callback, queue_size=10)
pub_bboxes = rospy.Publisher('bboxes',bbox_array,queue_size=10)


while (not rospy.is_shutdown()):
    rospy.spin()
