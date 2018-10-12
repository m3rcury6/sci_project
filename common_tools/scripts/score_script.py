#!/usr/bin/env python
'''
Author: Kris Gonzalez, Nitin Nayak
Objective: Script that can be used to generate results based off the three files
    each image should have (img, true labels, predictions). Recommended to
    export the results as a csv (comma separated values) file.

How to use file:
>> cd DesiredFolder/
>> rosrun common_tools score_script.py OutputFileName.csv

General layout of exported file:
    imgname | FP | FN | any IOU values
    ...     | .. | .. | ... , ... , ...

'''

import cv2
import numpy as np
import common_tools.lib_tools as b
import os
from sys import argv

if(len(argv) != 2):
    print 'ERROR: need output filename, exiting.'
    exit()

print 'Running in folder:',os.getcwd()
print 'Output filename:',argv[1]

# get list of unique filenames
names = [ifile.split('.')[0] for ifile in os.listdir('.') if '.jpg' in ifile]

f = file(argv[1],'w')
nl = '\n'
f.write('ImgName,FalsePositives,FalseNegatives,IOUs'+nl)

for iname in names:
    img=cv2.imread(iname+'.jpg')
    bbt=b.getBoxes(iname+'.txt',img.shape)
    bbp=b.getBoxes(iname+'.pred',img.shape)
    (i_iou,ifp,ifn) = b.calcEachIOU(bbt,bbp)
    print iname,'| FP:',ifp,'| FN:',ifn,'| nIOU:',len(i_iou)
    f.write(iname+'.jpg,')
    f.write(str(ifp)+','+str(ifn))
    for j in i_iou:
        f.write(','+str(j))
    f.write(nl)
f.close()
print 'Done writing results.\n'

# eof
