#!/usr/bin/env python
'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: Node that sends out an image, then listens for a count on the number
	of perceived cones in the image.

General Steps:
1. ensure read and write paths are functional
2. load and send images to test
3. wait for counter to come back (with maximum wait time?)
4. receive counter value and save to array / file (image name, cone detection)


NOTES:
sections marked as *.1 are for 'read' / initial items
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
print 'path:'
print read_path
if(not os.path.exists(read_path)):
	print "error, path below doesn't exist. creating..."
	print 'currently located at:',os.getcwd()
	os.mkdir(read_path)
os.chdir(read_path)
print 'currently located at : ',os.getcwd()


print 'kjgnote: here :)'
exit()

# 1.2: Ensure write path is available ##########################################
write_path = argv[2] # second argument from system
print 'path:'
print write_path
if(not os.path.exists(write_path)):
	print "error, path below doesn't exist. creating..."
	print write_path
	os.mkdir(write_path)
os.chdir(write_path)
print 'currently located at:',os.getcwd()

# 2.1: load all images to be output into single list ###########################
image_list = []
image = []
valid_images = [".jpg"]
for filename in os.listdir(read_path):
	ext = os.path.splitext(filename)[1]
	if ext.lower() not in valid_images:
		continue
	image_list.append(filename)

for val in range(len(image_list)):
	image.append(cv2.imread(str(image_list[val]), 1))
	'''
	TO VIEW EACH IMAGE remove this comment
	cv2.imshow(str(image_list[val]),image[val])
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	'''


# 2.2: create write file (csv) in order to log detections ######################
fname = "out_"+str(time.time())+".csv"
fout = file(fname,'w')
fout.write('imgname,cone_count\n')

# KJGNOTE: need to correct things after this point.


# 3.0: initialize node with counters, variables, etc ###########################
allCount=0
def call_result(combined):
	(imgname,ct)=combined.data.split(',')
	global allCount
	print "=",time.ctime(time.time()),"========="
	print "img:",imgname
	print "num:",ct
	fout.write(combined.data+'\n')
	allCount=allCount+int(ct)
	print "tot:",allCount
# def call_result

rospy.init_node('count_node')
# pub_txt = rospy.Publisher('talk_node',String,queue_size=10)
rospy.Subscriber('detector_result',String,call_result,queue_size=10)



val = 0	#kills ROS node once finished with all images
raw_input('Press ENTER to continue... ')
print 'publishing images...'




# 99: main loop ################################################################
while(not rospy.is_shutdown()):
	rospy.spin()
