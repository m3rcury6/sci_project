'''
Author: Kris Gonzalez
Objective: create sub-image files from training images. these sub-images are
    unmodified crops using the bounding boxes of each image. This script has
    been created to help convert darknet data into pytorch data for training
'''

import cv2
import os
from sys import argv
import common_tools.lib_tools as lib


# CONTROL SECTION =========================================

# startpath = 'temp_practice/'
startpath = 'imgs/'

file_list = 'test.txt'
out_path = 'test_subimgs/'
img_margin = 10
# file_list = 'test_mod.txt'
# file_list = 'train_mod.txt'
# out_path = 'test_subimgs'

# CONTROL SECTION END =====================================


if(len(argv)==2):
    # user specified a desired startpath. otherwise, use default
    startpath = argv[1]

os.chdir(startpath)
print 'dir:',os.getcwd()

# get list of file names (without extension)
# imgnames = [i.split(',')[0] for i in os.listdir('.') if '.jpg' in i]
# kjgnote: DONT want all files, just the files to train... need to get this from file

ftrain = file(file_list)

imgnames = [irow.split('.')[0] for irow in ftrain]
ftrain.close()
isubcount=0
for iname in imgnames:
    # load image, bounding boxes, and get subimgs
    img = cv2.imread(iname+'.jpg')
    fbox = iname+'.txt'
    # print fbox
    boxes = lib.getBoxes(fbox,img.shape,onlyBlue=False)
    # print len(boxes)
    i=0
    # lib.qs(img)
    for ibox in boxes:
        r1=max(ibox[1]-img_margin,0) # top edge
        r2=min(ibox[3]+img_margin,img.shape[0]) # bot edge
        c1=max(ibox[0]-img_margin,0) # left edge
        c2=min(ibox[2]+img_margin,img.shape[1]) # right edge
        # lib.qs(img[r1:r2,c1:c2],iname+' , '+str(i))
        subimg=img[r1:r2,c1:c2]
        subimg_path_and_name=out_path+iname+'_'+lib.pad(str(i),2,'0')+'.jpg'
        cv2.imwrite(subimg_path_and_name,subimg)
        i=i+1
        print 'created:',subimg_path_and_name
        isubcount=isubcount+1
    # for each box
# for each image
print isubcount,'images created'
print 'done'








# eof
