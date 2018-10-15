#!/usr/bin/env python
'''
Author: Kris Gonzalez
Objective: create sub-image files from darknet-formatted training images.
    these sub-images are unmodified crops using the bounding boxes of each
    image. This script has been created to help convert darknet data into HOG
    data for training.

arg1: SourcePath (containing images and yolo-based labels)
arg2: OutPath (where subimgs and pixel-format labels will be saved)


FUTURE IMPROVEMENTS:
* ability to select between yolo and pixx label
* use argparse in order to take command line arguments
* more generalization between format conversion...?
'''

import cv2
import os
from sys import argv
import common_tools.lib_tools as lb


# CONTROL SECTION =========================================

startpath = argv[1]
if(startpath[-1] != '/'): startpath+='/'
outpath = argv[2]
img_margin = 10 # how much margin to include around label
# outpath = 'test_subimgs'

# CONTROL SECTION END =====================================
print ''

# get list of file names (without extension)
imgnames = [i.split('.')[0] for i in os.listdir(startpath) if '.jpg' in i]

print imgnames

# print 'stopping here for debugging.';exit()

isubcount=0
for iname in imgnames:
    # load image, bounding boxes, and get subimgs
    img = cv2.imread(startpath+iname+'.jpg')
    boxes = lb.getBoxes(startpath+iname+'.txt',img.shape)
    # at this point, have image and all bounding boxes loaded.

    i=0
    for ibox in boxes:
        r1=max(ibox[1]-img_margin,0)            # top edge
        r2=min(ibox[3]+img_margin,img.shape[0]) # bot edge
        c1=max(ibox[0]-img_margin,0)            # left edge
        c2=min(ibox[2]+img_margin,img.shape[1]) # right edge
        subimg=img[r1:r2,c1:c2]
        ibb = [img_margin,img_margin,subimg.shape[1]-img_margin,subimg.shape[0]-img_margin]
        ibb_str=[str(ival) for ival in ibb]
        subimg_name=outpath+iname+'_'+lb.pad(str(i),2,'0')
        cv2.imwrite(subimg_name+'.jpg',subimg)
        f=file(subimg_name+'.hog','w')
        f.write(' '.join(ibb_str)+'\n')
        f.close()
        i=i+1
        print 'created:',subimg_name
        isubcount=isubcount+1
    # for each box
# for each image
print isubcount,'images created'
print 'done'








# eof
