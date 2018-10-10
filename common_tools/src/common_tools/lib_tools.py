'''
Authors: Kris Gonzalez, Nitin Nayak
Objective: This file (module) provides central, simplified location to keep
    functions and classes that are used across multiple nodes or code files. In
    this way, can avoid repeating work or overlapping functions.

Usage: EXAMPLE OF HOW TO CALL THIS MODULE IN OTHER CODE / SHELLS:
>>> import common_tools.lib_tools as b

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

def qs(img,title='CLOSE WITH KEYBOARD'):
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
    k = np.diag([cw,rh,cw,rh]) # almost works.
    # k = np.diag([1,rh,rh,cw,cw]) # will note color for now

    if('.jpg' in boxfile): boxfile = boxfile.split('.')[0] # if img, keep only name
    if('.' not in boxfile): boxfile = boxfile+'.txt' # if don't have a filetype, add .txt

    f = file(boxfile)
    for irow_temp in f:
        # # kjgnote: will not worry about classes for now, just collect all.
        # color = int(irow_temp[0])
        # if(onlyBlue):
        #     if(color==1):
        irow=irow_temp[:-1] # remove '\n' from txt
        # will not care about color for now
        line = irow.split(' ')[1:] # drop color info, in future only care for blue
        line = np.array([float(i) for i in line]) # now have values in percentage
        out = np.matmul(line,k) # convert percentages to pixel locations (float)
        # at this point, have out as array: [xc,yc,w,h] in pixels
        x1=out[0]-out[2]/2.0
        x2=out[0]+out[2]/2.0
        y1=out[1]-out[3]/2.0
        y2=out[1]+out[3]/2.0
        new=np.array([x1,y1,x2,y2])
        boxes=np.row_stack((boxes,new))
    f.close()
    return boxes.astype(int) # return as integer values

def putBoxes(img,box_array,txtCount=False,color=(250,0,0)):
    ''' Objective: Return image with augmented information

        ARGUMENTS:
        img: BGR image from cv2
        box_array: numpy array, should have format [x1,y1,x2,y2] as Nx4 array
        txtCount: boolean, decide if should have text overlay as well (default
            False)
    '''
    icount=-1
    # color notes:
    BLU=(250,0,0)
    WHI=(250,250,250)

    imgnew=img.copy()
    for ibox in box_array:
        icount=icount+1
        imgnew=cv2.rectangle(imgnew,tuple(ibox[:2]),tuple(ibox[2:]),color)
        if(txtCount==True):
            FONT=cv2.FONT_HERSHEY_PLAIN
            imgnew=cv2.putText(imgnew,str(icount),tuple(ibox[:2]+np.array([0,-2])),FONT,1,color)
        # once complete, return image
    return imgnew
#

def countBoxes(boxfile):
    ''' Objective: Return number of blue cones in a single file

        ARGUMENTS:
        boxfile: filename (e.g. 'img3.txt') of bounding box file
    '''
    count=0

    # following enables file extension to be wrong, so long as filename is correct
    if('.jpg' in boxfile): boxfile = boxfile.split('.')[0] # if img, keep only name
    if('.' not in boxfile): boxfile = boxfile+'.txt' # if don't have a filetype, add .txt
    f=file(boxfile)
    for irow in f:
        # print irow
        if(irow[0]=='1'):
            # row has a blue cone, increment
            count=count+1
    # print boxfile,':',str(count)
    f.close()
    return count

def folderCountAll(show=False):
    ''' in a single folder, check each image and count all boxes. return total.
    '''
    tot=0
    for ifile in os.listdir('.'):
        if('.txt' in ifile):
            itot = countBoxes(ifile)
            tot+=itot
            if(show==True):
                print ifile,itot
    return tot

class ImgTextPair(object):
    ''' Objective: container class for an image / text pair. image will be a cv2
        image class. text will be an Nx4 numpy array of bounding boxes. note
        that a single pair can contain multiple bounding boxes.
    Assumptions:
        * This is class intends the user to access member variables directly (not
        just through member functions)

    '''
    def __init__(self, img=np.zeros((3,3,3),dtype=np.uint8), boxes=np.ndarray((0,4)) ):
        self.img = img # default initialize as blank
        self.boxes = boxes # default initialize as empty
    # at this point, have all items defined...?

    def _LoadFromFile(self,filename):
        ''' Objective: If want to directly load an image from storage, can run
            this. the danger with this function is that a user may accidentally
            load the same image/text combination twice through oversight in a
            forloop.
        '''
        if('.' in filename):
            fname = filename.split('.')[0]
        else:
            fname = filename
        self.img = cv2.imread(fname+'.jpg')
        self.boxes = getBoxes(fname+'.txt',self.img.shape)
    # def _LoadFromFile
# class ImgTextPair

def pad(text,strLen,char=' ',side='L'):
    ''' Objective: provide easy, powerful padding tool for
        text. vars:
        'text': the text to pad
        'strLen': maximum length of final string
        'char': padding character, default ' '
        'side': side to pad, default left. options: L, R
    '''
    if(len(text)<strLen):
        if(side=='R'):
            return pad(text+char,strLen,char,side)
        else:
            return pad(char+text,strLen,char,side)
    else:
        return text
# def pad

def getIOU(boxA, boxB):
    ''' given two numpy array bounding boxes (bboxes), return
        the IOU (%) value of the two.
    ASSUMPTIONS:
        * will assume that box format is (x1,y1,x2,y2) in pixels
    SOURCE: A. Rosebrock, www.pyimagesearch.com
    '''
    # determine the (x, y)-coordinates of the intersection rectangle
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou
# def getIOU

def getCTC(boxA,boxB):
    ''' Calculate center-to-center distance between two bounding boxes.
    '''
    xc1=(boxA[0]+boxA[2])/2.0
    yc1=(boxA[1]+boxA[3])/2.0
    xc2=(boxB[0]+boxB[2])/2.0
    yc2=(boxB[1]+boxB[3])/2.0
    return ( (xc2-xc1)**2 + (yc2-yc1)**2 )**0.5
# def getCTC



def calcEachIOU(boxes_true,boxes_pred,iou_thresh=0.1):
    ''' Simplifies task of comparing the IOU between a set of true bboxes and
        a set of predicted bboxes. with two forloops nested in a while loop,
        the larger data set is the the outer forloop. for each pair of bboxes
        with the highest IOU larger than some threshold, the IOU is stored and
        the pair are removed from their respective lists.

        When there are no more pairs to find, the number of false positives (FP)
        and false negatives (FP) are counted and the results are returned as a
        tuple: (numpy vector of IOU, scalar FP, scalar FN).

    KJGNOTE: this function still having issues calculating things correctly
    '''

    bbt=np.copy(boxes_true) # ensure no changes are made to original arrays
    bbp=np.copy(boxes_pred)
    if(len(bbt)>len(bbp)):
        # more true boxes than prediction boxes
        set1=bbt # outer loop must be larger than inner loop
        set2=bbp
    else:
        set1=bbp
        set2=bbt
    keep_looking=True
    iou_list=[]
    while(keep_looking):
        # may still be able to find more pairs
        best_i=0
        best_IOU=0.0
        for i in range(len(set1)):
            best_j=0
            for j in range(len(set2)):
                new_IOU=getIOU(set1[i],set2[j])
                if(new_IOU>best_IOU):
                    best_IOU=new_IOU
                    best_i=i
                    best_j=j
            # j_forloop
        # i_forloop
        if(best_IOU>iou_thresh):
            # able to find two bboxes that overlap. remove them from lists & restart
            set1=np.delete(set1,best_i,0)
            set2=np.delete(set2,best_j,0)
            iou_list.append(best_IOU)
            keep_looking=True
        else:
            # couldn't find any more pairs
            keep_looking=False
    # end of while loop
    nFP = len(bbp)-len(iou_list)
    nFN = len(bbt)-len(iou_list)
    return (np.array(iou_list),nFP,nFN) # return tuple
# def getEachIOU

def pixx2dnet(bboxes_in_pixel,img_shape):
    ''' Convert bounding boxes that are in pixel format (x1,y1,x2,y2) to
    darknet format (pxc,pyc,pw,ph). note 'p' means percentage, 'c' means center

    NOTE:
    '''
    bboxes_in_dnet=[]
    HT=np.double(img_shape[0])
    WD=np.double(img_shape[1])
    for ibox in bboxes_in_pixel:
        w=ibox[2]-ibox[0]
        h=ibox[3]-ibox[1]
        xc=ibox[0]+(w)/2.0
        yc=ibox[1]+(h)/2.0
        pw=np.double(w)/WD
        ph=np.double(h)/HT
        pxc=xc/WD
        pyc=yc/HT
        bboxes_in_dnet.append([pxc,pyc,pw,ph])
    return np.array(bboxes_in_dnet)
# def pixx2dnet

def dnet2pixx(bboxes_in_dnet,img_shape):
    ''' Convert bounding boxes that are in darknet format (pxc,pyc,pw,ph) to
        pixel format (x1,y1,x2,y2). note 'p' means percentage, 'c' means center
    '''
    bboxes_in_pixel=[]
    HT=float(img_shape[0])
    WD=float(img_shape[1])
    for ibox in bboxes_in_dnet:
        xc=ibox[0]*WD
        yc=ibox[1]*HT
        w=ibox[2]*WD
        h=ibox[3]*HT
        x1=int(xc-w/2.0)
        y1=int(yc-w/2.0)-3 # empirically proven to have less error
        x2=int(xc+w/2.0) # empirically proven to have less error
        y2=int(yc+h/2.0) # empirically proven to have less error
        bboxes_in_pixel.append([x1,y1,x2,y2])
    return np.array(bboxes_in_pixel)
# def dnet2pixx

def saveBoxes(filename,bboxes):
    ''' Export array of bounding boxes to text file. extension should be *.pred,
        and format of bboxes should be in pixels (dnet). function AUTOMATICALLY
        checks if bboxes are in correct format, and will auto-format if
        necessary.
    ASSUMPTIONS:
    * only working with one class ('blue_cones'), so thus will have class column
        as '0'
    * to avoid tedium of saving two separate bbox formats (pixx and dnet), will
    simply export as dnet
    * process is not lossless. conversion between pixx and dnet DOES produce a
        little noise, but empirically tested to be negligible
    '''
    sample=bboxes[0,0]
    if(float(sample)>1.0):
        # bbox format not already in dnet, give error
        print 'ERROR: bboxes in pixx format, require dnet format. exiting'
    else:
        f=open(filename,'w')
        for irow in bboxes:
            s=[str(i) for i in irow]
            f.write('0 '+' '.join(s)+'\n')
        # have written everything
        f.close()
        print 'export complete:',filename
# def saveBoxes


#test

#eof
