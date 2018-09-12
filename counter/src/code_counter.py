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
fout.write('colA,colB')
fout.close()

def call_inc(data):
	print "i heard something",time.time()

rospy.init_node('count_node')
# pub_txt = rospy.Publisher('talk_node',String,queue_size=10)
rospy.Subscriber('count_topic',UInt8,call_inc,queue_size=10)
while(not rospy.is_shutdown()):
	rospy.spin()
