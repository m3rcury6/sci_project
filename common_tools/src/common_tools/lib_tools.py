'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: This file (module) provides central, simplified location to keep
    functions and classes that are used across multiple nodes or code files. In
    this way, can avoid repeating work or overlapping functions.

Usage: EXAMPLE OF HOW TO CALL THIS MODULE IN OTHER CODE / SHELLS:
>>> import common_tools.lib_tools as lib

'''

# INITIALIZATIONS ##############################################################
    # recommend only to import modules that are required for functioning of \
    # functions and classes in this file
import numpy as np
import cv2
import os

# FUNCTIONS ####################################################################
def pyt(arr):
    ''' given a list of values, return the 2-Norm
    '''
    sum=0.0
    for i in arr:
        sum=sum+i**2
    return sum**0.5

    cv2.imshow(title,img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def getBoxes(boxfile,img_shape,onlyBlue=True):
    ''' Objective: Return pixel boundaries of boxes of a particular image.
        general input layout seems to be xc,yc,w,h (c=center)
        output shall be: x1,y1,x2,y2

        ARGUMENTS:
        boxfile: filename (e.g. 'img3.txt') of bounding box file
        img_shape: tuple of file dimensions (e.g. img3.shape or (640,480,3) )
        onlyBlue: in output array, include only blue cones. (default True)
    '''
    boxes=np.array([[],[],[],[]]).transpose() # 0x4 initialization
    # boxes=np.array([[],[],[],[],[]]).transpose() # 0x5 initialization
    rh=img_shape[0]
    cw=img_shape[1]
    k = np.diag([rh,rh,cw,cw]) # doesn't seem to be right
    k = np.diag([cw,rh,cw,rh]) # almost works.
    # k = np.diag([1,rh,rh,cw,cw]) # will note color for now
    f = file(boxfile)
    for irow_temp in f:
        color = int(irow_temp[0])
        if(onlyBlue):
            if(color==1):
                irow=irow_temp[:-1] # remove '\n' from txt
                # will not care about color for now
                line = irow.split(' ')[1:] # drop color info, in future only care for blue
                line = np.array([float(i) for i in line]) # now have values in percentage
                out = np.matmul(line,k) # convert percentages to pixel locations (float)\
                # at this point, have out as array: [xc,yc,w,h] in pixels
                x1=out[0]-out[2]/2.0
                x2=out[0]+out[2]/2.0
                y1=out[1]-out[3]/2.0
                y2=out[1]+out[3]/2.0
                new=np.array([x1,y1,x2,y2])
                boxes=np.row_stack((boxes,new))
    f.close()
    return boxes.astype(int) # return as integer values

def putBoxes(img,box_array,txtCount=False):
    ''' Objective: Return image with augmented information

        ARGUMENTS:
        img: BGR image from cv2
        box_array: numpy array, should have format [x1,y1,x2,y2] as Nx4 array
        txtCount: boolean, decide if should have text overlay as well (default
            False)
    '''
    icount=-1
    BLU=(250,0,0)
    WHI=(250,250,250)
    imgnew=img.copy()
    for ibox in box_array:
        icount=icount+1
        imgnew=cv2.rectangle(imgnew,tuple(ibox[:2]),tuple(ibox[2:]),BLU)
        if(txtCount==True):
            FONT=cv2.FONT_HERSHEY_PLAIN
            imgnew=cv2.putText(imgnew,str(icount),tuple(ibox[:2]),FONT,1,WHI)
        # once complete, return image
    return imgnew
#

def countBoxes(boxfile):
    ''' Objective: Return number of blue cones in a single file

        ARGUMENTS:
        boxfile: filename (e.g. 'img3.txt') of bounding box file
    '''
    count=0
    f=file(boxfile)
    for irow in f:
        # print irow
        if(irow[0]=='1'):
            # row has a blue cone, increment
            count=count+1
    # print boxfile,':',str(count)
    f.close()
    return count

def folderCountAll():
    ''' in a single folder, check each image and count all boxes. return total.
    '''
    total=0
    for ifile in os.listdir('.'):
        total=total+countBoxes(ifile)
    return total
