#!/usr/bin/env python
'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: simple listener for results from an attempt at detecting cones.


'''

import rospy
import time
from sys import argv
from std_msgs.msg import String
from std_msgs.msg import UInt8
import os

# 1.0: Ensure write path is available ##########################################
write_path = argv[1]
print 'path:'
print write_path
if(not os.path.exists(write_path)):
	print "error, path below doesn't exist. creating..."
	print write_path
	os.mkdir(write_path)
os.chdir(write_path)
print 'currently located at:',os.getcwd()

# 2.0: create write file (csv) in order to log detections ######################
fname = "out_"+str(time.time())+".csv"
fout = file(fname,'w')
fout.write('imgname,cone_count\n')


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

# 99: main loop ################################################################
while(not rospy.is_shutdown()):
	rospy.spin()
