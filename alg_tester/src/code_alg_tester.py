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
from cv_bridge import CvBridge
from sys import argv
import common_tools.lib_tools as lib

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import UInt8

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

# 2.2: create write file (csv) in order to log detections ######################
fname = "out_"+str(time.time())+".csv"
fout = file(fname,'w')
fout.write('imgname,cone_count\n')


# 3.0: initialize node with counters, variables, etc ###########################
allCount=0
flag_go_next=False

def call_result(combined):
    # expecting data in string format as "imgname,count"
    (imgname,ct)=combined.data.split(',')
    global allCount
    global flag_go_next
    print "=",time.ctime(time.time()),"========="
    print "img:",imgname
    print "num:",ct
    fout.write(combined.data+'\n') # write out to file
    allCount=allCount+int(ct)
    print "tot:",allCount
    flag_go_next=True
# def call_result

rospy.init_node('alg_tester_node')
# pub_txt = rospy.Publisher('talk_node',String,queue_size=10)
rospy.Subscriber('detector_result',String,call_result,queue_size=10)
pub_image = rospy.Publisher('images', Image, queue_size=10)
pub_image_name = rospy.Publisher('image_name', String, queue_size=10)


# raw_input('Press ENTER to continue... ') # disabling this for debugging
print 'publishing images...'

# # KJGNOTE: need to correct things after this point.
# print 'kjgnote: here :)'
# exit()

val = 0    #kills ROS node once finished with all images
max_wait_time=3.0 #seconds

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
