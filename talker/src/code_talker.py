#!/usr/bin/env python

'''
Objective: hello world style of ros and github initialization
'''

import rospy
import time
from sys import argv
from std_msgs.msg import String
import common_tools.lib_tools as lib

rospy.init_node('args_node')
pub_txt = rospy.Publisher('talk_node',String,queue_size=10)
a=3.0
b=4.0


while(not rospy.is_shutdown()):
    print (a*a+b*b)**0.5
    print 'new:',lib.pyt([a,b])
    pub_txt.publish('hello world'+str(time.time()))
    time.sleep(1)
