#!/usr/bin/env python
'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: Node that sends out an image, then listens for a count on the number
    of perceived cones in the image.

General Steps:
1.1 check read path
1.2 load into memory all images to test
2.1 check write path
2.2 create new csv file to record results
3.0 wait for counter to come back (with maximum wait time?)
4.0 receive counter value and save to array / file (image name, cone detection)


'''

# 0.0 Initializations ##########################################################
import rospy
import time
import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from sys import argv
import common_tools.lib_tools as b

from ai_darknet.msg import bbox
from ai_darknet.msg import bbox_array
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import UInt8

def msg2dnet(msg_array):
    ''' take an array of bbox messages and convert to numpy array of bboxes in
        dnet format
    * bbox format: bb[i]: bb[i].prob/pxc/pyc/pw/ph
    * dnet format: bb[i] = [pxc,pyc,pw,ph]
    '''
    bb_dnet=[]
    for ibb in msg_array:
        bb_dnet.append([ibb.x,ibb.y,ibb.w,ibb.h])
    return np.array(bb_dnet)

# 1.1 Ensure read path is available ############################################
read_path = argv[1]
if(not os.path.exists(read_path)):
    print "error, path below doesn't exist. creating..."
    os.mkdir(read_path)
else:
    print 'Success: read path found.'
os.chdir(read_path)
print 'Located at:',os.getcwd()

# 1.2: load all images to be output into single list ###########################
image_list = []
image = []
valid_images = [".jpg"]
for filename in os.listdir(read_path):
    ext = os.path.splitext(filename)[1]
    if ext.lower() not in valid_images:
        continue
    image_list.append(filename)
# at this point, have list of image names
image_list.sort()

for val in range(len(image_list)):
    image.append(cv2.imread(str(image_list[val]), 1))
# at this point, have list of image content (cv2 arrays)

# 2.1: Ensure write path is available ##########################################
write_path = argv[2] # second argument from system
if(not os.path.exists(write_path)):
    print "error, path below doesn't exist. creating..."
    os.mkdir(write_path)
else:
    print 'Success: write path found.'
os.chdir(write_path)
print 'Located at:',os.getcwd()

# kjgnote: sect 2.2 not needed
# 2.2: create write file (csv) in order to log detections ######################
# fname = "out_"+str(time.time())+".csv"
# fout = file(fname,'w')
# fout.write('imgname,cone_count\n')


# 3.0: initialize node with counters, variables, etc ###########################
allCount=0
flag_go_next=False

def call_bboxes(dat):
    ''' mainly, just want to be able to get the
        bboxes in the right format to save'''

    imgname = dat.header.frame_id
    bb=dat.bboxes # get all bounding boxes into one array of object
    bb2=msg2dnet(bb)

    print 'img:',imgname
    predname = imgname.split('.')[0]+'.pred'

    # # to be re-added: saving to output
    print 'saving to folder...'
    if(len(bb2)>0):
        b.saveBoxes(write_path+predname,bb2)
    else:
        f=file(write_path+predname,'w')
        f.close()
    global flag_go_next
    flag_go_next=True


rospy.init_node('alg_tester_node')
# pub_txt = rospy.Publisher('talk_node',String,queue_size=10)
# rospy.Subscriber('detector_result',String,call_string,queue_size=10)
rospy.Subscriber('bboxes',bbox_array,call_bboxes,queue_size=1)

pub_image = rospy.Publisher('images', Image, queue_size=10)
pub_image_name = rospy.Publisher('image_name', String, queue_size=10)


# raw_input('Press ENTER to continue... ') # disabling this for debugging
print 'publishing images...'

# # KJGNOTE: need to correct things after this point.
# print 'kjgnote: here :)'
# exit()

val = 0    #kills ROS node once finished with all images
max_wait_time=3.0 #seconds

# going to try publishing initial image to get things going...
# not sure why, but first image published seems to not send or be lost in buffer
print 'ESTABLISHING CONNECTION...'
pub_image_name.publish(str(image_list[val]))
time.sleep(0.01)# small delay to ensure name is published first
pub_image.publish(CvBridge().cv2_to_imgmsg(image[val],"bgr8"))

time.sleep(3)
allCount=0 # reset counter after initial image


# 99: main loop ################################################################
while(not rospy.is_shutdown() and val < len(image_list)):
    # want to send out an image, wait (with timelimit), then take in counter
    print 'publishing:',str(image_list[val])
    pub_image_name.publish(str(image_list[val]))
    time.sleep(0.01)# small delay to ensure name is published first
    pub_image.publish(CvBridge().cv2_to_imgmsg(image[val],"bgr8"))
    flag_go_next=False
    starttime=time.time()
    while(not flag_go_next):
        # waiting for next thing to do
        if(time.time()-starttime>max_wait_time):
            # have waited n seconds, move on to next image.
            print 'time limit reached, moving to next image...  '
            flag_go_next=True

    # at this point, will move onto next image
    val=val+1
# main while loop
print 'all images loaded. node closing...'


# eof
