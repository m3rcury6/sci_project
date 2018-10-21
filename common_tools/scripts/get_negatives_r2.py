#!/usr/bin/env python
'''
Objective: Get only negative sub-images from a single image.

General Steps:
* get list of images to manipulate
* for each image:
    * load and get dimensions
    * enlarge desired x/y subimg dimensions to grab more data with less subimgs
    * get initial set of bboxes
    * remove bboxes that have IOU>0 with a true bbox
    * export all remaining bbox crops to subimgs folder

How to Use:
(copy python script to desired folder)
>> cd FolderWithImagesAndLabels
>> mkdir OutputFolder
>> python get_negatives.py OutputFolder/
(delete python script when finished)

input arguments:
arg1: OutputFolder

'''

# initializations
import numpy as np
import cv2
import common_tools.lib_tools as b
import os
from sys import argv





# config stuff =====================================
des_y=128
des_x=64
# stride of each dim is chosen to be about 1/3 of calculated dimensions

# config stuff end ================================

# put output folder into variable
if(len(argv)!=2):
    print('ERROR: need single input argument: output folder\n')
    exit()
outfolder=argv[1]

# load set of images to use
imglist=[ifile for ifile in os.listdir('.') if '.jpg' in ifile]
allCount=0

for imgfile in imglist:
    print('========================================')
    print('Loading '+imgfile)
    # load image
    img=cv2.imread(imgfile) # *.shape returns (ht,wd,channels)

    # get bbox dimension to use everywhere, special version
    dy2=des_y+0
    dx2=des_x+0
    while(img.shape[0]%dy2 != 0): dy2+=1
    while(img.shape[1]%dx2 != 0): dx2+=1
    dH=max(dy2-1,dx2*2-1)
    if(dH%2!=0): dH-=1
    dW=dH/2

    # at this point, have some fixed size that is a multiple of (des_x,des_y)
    # now want to get number of crops per dimension, given a stride (step)
    # note: have dH and dW

    # old calculation
    nRowcuts=img.shape[0]/dH # seems to work same as mod()
    nColcuts=img.shape[1]/dW

    strideW = dW/3
    strideH = dH/3
    print('stride (W,H) is: '+str((strideW,strideH)))
    # newCalc
    nRowcuts=(img.shape[0]-dH)/strideH
    nColcuts=(img.shape[1]-dW)/strideW

    # first, generate the proposed bbox crops, then remove wrong ones
    bb_crop=[]

    for i in range(nRowcuts): # row first, column second forloops
        for j in range(nColcuts):
            bb_crop.append([j*strideW,i*strideH,j*strideW+dW,i*strideH+dH])
    bb_crop=np.array(bb_crop)

    filetxt=imgfile.split(',')[0]+'.txt'
    bbt=b.getBoxes(filetxt,img.shape)
    # at this point, need to check every ibbt to remove bboxes that have IOU>0
    for ibbt in bbt:
        j_rm_list=[]
        for j in range(len(bb_crop)):
            iou = b.getIOU(ibbt,bb_crop[j])
            if(iou>0.0): #not recommended to change this value from 0.0
                j_rm_list.append(j)
        # in each loop, remove any rows that had bbox overlap
        bb_crop = np.delete(bb_crop,j_rm_list,0)
    # # quick check to see results
    # temp=b.putBoxes(img,bb_crop)
    # temp = b.putBoxes(temp,[bb_crop[-1]],color=(0,0,255))
    # b.qs(temp)

    print('Number of negative boxes:',len(bb_crop))
    allCount+=len(bb_crop)
    i=0
    for ibb in bb_crop:
        img2=img[ibb[1]:ibb[3],ibb[0]:ibb[2]] # needs to be in (y1:y2,x1:x2)
        img2=cv2.resize(img2,(des_x,des_y)) # once again resize to proper
        savename=outfolder+imgfile.split('.')[0]+'-'+b.pad(str(i),3,'0')+'.jpg'
        cv2.imwrite(savename,img2)
        i+=1

print('Subimg export complete. Total images created:',allCount)













# eof
