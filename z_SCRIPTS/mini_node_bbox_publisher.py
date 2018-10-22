'''
objective: mini node that only publishes garbage data, but as a demo for how to
    publish bbox arrays

'''

import rospy
import cv2
import numpy as np
import common_tools.lib_tools as lb
from ai_darknet.msg import bbox
from ai_darknet.msg import bbox_array
import time


rospy.init_node('bbox_mininode')
pub_bboxes = rospy.Publisher('bboxes',bbox_array,queue_size=10)




i=0
while (not rospy.is_shutdown()):
    imgname = 'img_'+lb.pad(str(i),3,'0')+'.jpg'; i+=1
    print 'publishing:',imgname

    msgs = bbox_array() # generate new, empty custom message array
    # will publish 3 random bboxes each loop
    bb = np.random.rand(4,4) # dummy data
    for irow in bb:
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


    time.sleep(1)
# main loop








#eof
