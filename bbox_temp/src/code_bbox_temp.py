'''
objective: just want to quicky detect, capture, and save published bbox information.

note: will also need image information, which is embedded in bboxmsg.header.frame_id

'''

import cv2
import numpy as np
import rospy
import common_tools.lib_tools as b
from ai_darknet.msg import bbox
from ai_darknet.msg import bbox_array

outfolder='/home/fstw/vision_project/catkin_ws/src/output/'
imgfolder='/home/fstw/vision_project/catkin_ws/src/input/'

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


# def msg2net

def call_bboxes(dat):
    ''' mainly, just want to be able to get the
        bboxes in the right format to save'''

    imgname = dat.header.frame_id
    bb=dat.bboxes # get all bounding boxes into one array of object
    bb2=msg2dnet(bb)

    print 'img:',imgname
    predname = imgname.split('.')[0]+'.pred'

    print 'saving to folder...'
    b.saveB


rospy.init_node('temp_bboxes_node')
rospy.Subscriber('bboxes',bbox_array,call_bboxes,queue_size=1)
print 'node started.'
while(not rospy.is_shutdown()):
    rospy.spin()





#eof
