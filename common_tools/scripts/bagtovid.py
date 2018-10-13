'''
Author: Kris Gonzalez
Objective: Given a rosbag, convert to video file (or do whatever else you want
    with it to extract frames)

arg1 = BagName
arg2 = TopicName (still not sure if need '/' or not)
arg3 = OutputFileName.avi (must have .avi at end)
arg4 = swap? (1 means yes. default is 0 or leaving blank)
'''

import cv2
import numpy as np
import rosbag
from sys import argv
from cv_bridge import CvBridge, CvBridgeError #convert cv2<>rosimg
from sensor_msgs.msg import Image

swap=False
if(argv[4] == '1'):
    swap=True

# initialize video source
bag = rosbag.Bag(argv[1]) # open bag to read

# KJGNOTE: IN READ_MESSAGES, NEED '/'. DON'T NEED FOR GET_MESSAGE_COUNT
flag_firstloop=True
for irosmsg in bag.read_messages(topics=[argv[2]]):
    imsg = irosmsg[1]
    if(flag_firstloop):
        # first loop, need to initialize everything, because need resolution
        cv_frame = CvBridge().imgmsg_to_cv2(imsg)
        (ht,wd) = cv_frame.shape[:2]
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # control filename, 'fourcc' format, fps, and resolution
        out = cv2.VideoWriter(argv[3],fourcc, 30.0, (wd,ht))
        flag_firstloop=False # ensure doesn't run again
    # first loop initialization
    cv_frame=CvBridge().imgmsg_to_cv2(imsg)
    if(swap==True):
        # manually convert to new colorspace
        img=np.copy(cv_frame)
        tempB=np.copy(img[:,:,0])
        img[:,:,0] = img[:,:,2]
        img[:,:,2] = tempB
    else:
        img=cv_frame
    # write results out
    out.write(img)
    cv2.imshow('frame',img) # show what's currently being processed
    if cv2.waitKey(1) & 0xFF == ord('0'):
        break
# main loop

bag.close()
out.release()


# eof
