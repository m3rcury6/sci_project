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
    rh=np.double(img_shape[0])
    cw=np.double(img_shape[1])
    # k = np.diag([cw,rh,cw,rh])
    k = [cw,rh,cw,rh]
    # k = np.diag([1,rh,rh,cw,cw]) # will note color for now

    if('.jpg' in boxfile): boxfile = boxfile.split('.')[0] # if img, keep only name
    if('.' not in boxfile): boxfile = boxfile+'.txt' # if don't have a filetype, add .txt

    f = file(boxfile)
    for irow_temp in f:

        irow=irow_temp[:-1] # remove '\n' from txt
        # will not care about color for now
        line = irow.split(' ')[1:] # drop class info, assumes always blue
        z=[]
        for i in range(len(line)):
            z.append(np.double(line[i])*k[i])
        # at this point, have z as array: [xc,yc,w,h] in pixels
        x1=z[0]-z[2]/2.0
        x2=z[0]+z[2]/2.0
        y1=z[1]-z[3]/2.0
        y2=z[1]+z[3]/2.0
        new=np.array([x1,y1,x2,y2])
        boxes=np.row_stack((boxes,new))
    f.close()
    return boxes.astype(int) # return as integer values

def putBoxes(img,box_array,txtCount=False,color=(250,0,0),width=2):
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
        imgnew=cv2.rectangle(imgnew,tuple(ibox[:2]),tuple(ibox[2:]),color,thickness = width)
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

def calcEachIOU(bbt,bbp,thresh=0.0):
    ''' Simplifies task of comparing the IOU between a set of true bboxes and
        a set of predicted bboxes. With two forloops nested in a while loop,
        the larger data set is the the outer forloop. for each pair of bboxes
        with the highest IOU larger than some threshold, the IOU is stored and
        the pair are removed from their respective lists.

        When there are no more pairs to find, the number of false positives (FP)
        and false negatives (FP) are counted and the results are returned
        as a tuple: (numpy vector of IOU, scalar FP, scalar FN).
    '''
    best_iou=1 # initializer
    iset=range(len(bbt)) # initialize search set (helps traceability)
    jset=range(len(bbp))
    ncount=0 # debugging
    iou_set=[]
    iou_and_tp_pair=[] # currently unused
    while(best_iou > 0):
        best_iou = 0.0
        ibest = 0
        jbest = 0
        for i in iset:
            for j in jset:
                iou = getIOU(bbt[i],bbp[j]) # calculate IOU (b/t 1.0 and 0.0)
                if(iou>best_iou):
                    # found a better matching pair, save new info
                    best_iou = iou
                    ibest = i
                    jbest = j
        # nested forloop to find best iou pair between two indices
        if(best_iou>thresh):
            # debugging
            # ncount+=1; print '= item',ncount,'============='
            # print '(true,pred):',(ibest,jbest); print 'score:',best_iou

            # save results of loop & remove indices
            iou_set.append(best_iou)
            iou_and_tp_pair.append([best_iou,ibest,jbest]) # currently unused
            iset.remove(ibest)
            jset.remove(jbest)
        # if-iou greater than threshold
    # while-loop, repeat until no longer find more

    # once complete, find FP and FN and exit
    FalsePositives=len(bbp)-len(iou_set) # more predictions than pairs
    FalseNegatives=len(bbt)-len(iou_set) # more true bboxes than pairs
    ioutp_pairs=np.array(iou_and_tp_pair) # unused as of now

    return (np.array(iou_set),FalsePositives,FalseNegatives,ioutp_pairs)
# def getEachIOU

def pixx2dnet(bboxes_in_pixel,img_shape):
    ''' Convert bounding boxes that are in pixel format (x1,y1,x2,y2) to
    darknet format (pxc,pyc,pw,ph). note 'p' means percentage, 'c' means center
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

def load_all(name,showNumbers=True,display=False):
    ''' Simple tool to load the three files of an image. can optionally display
            them. Loads *jpg, *txt, and *pred files from same basename

        Arguments:
        * name: string of desired file, without extension. e.g. "IMAG1516"
        * display: boolean, control if img2 is displayed. default=False

        Returns tuple: (bbt, bbp, img, img2)
        * bbt: true bounding bboxes
        * bbp: predicted bounding boxes
        * img: original image
        * img2: image with all bboxes overlaid
    '''
    img=cv2.imread(name+'.jpg')
    bbt=getBoxes(name+'.txt',img.shape)
    bbp=getBoxes(name+'.pred',img.shape)
    img2=putBoxes(img,bbt,txtCount=showNumbers)
    img2=putBoxes(img2,bbp,txtCount=showNumbers,color=(0,0,250))
    if(display==True):
        qs(img2)
    return (bbt,bbp,img,img2)
# def compare








#eof
