
# KJGNOTE: this script seems to be working, but not fully tested.

import os
import random
import common_tools.lib_tools as k

imgdirectory = 'training_all'
path_data = '/data/obj/cone_imgs/'

pct_test = 10 # percentage of cones to test on

def getCount(fname):
    ''' return number of lines in file, assuming all lines are for blue cone class'''
    f=file(fname)
    count = len([irow for irow in f])
    f.close()
    return count

# open text files in starting directory, outside of img data
ftrain = file('train.txt','w')
ftest = file('test.txt','w')

# go into data directory
os.chdir(imgdirectory)

# get count of total data
total_labels = k.folderCountAll()
ntest = int(round(total_labels*float(pct_test)/100.0)) # get total number of cones to test on

# get a randomized list of cone data to test on
flist = [i for i in os.listdir('.') if '.txt' in i]
random.shuffle(flist)

# iterate through list and sort files into training or test
current_test = 0

for ifile in flist:
    img_name = ifile.split('.')[0]+'.jpg'
    if(current_test<ntest):
        ftest.write(path_data+img_name+'\n') # designate img to test data set
        current_test = current_test + getCount(ifile) # update number of cones in test set
    else:
        # write the remaining cones to the training set
        ftrain.write(path_data+img_name+'\n') # designate img to test data set
# forloop

# close text files
ftest.close()
ftrain.close()

print 'total number of cones in set:', total_labels
print 'number of cones in test set: ', current_test

print 'done'
