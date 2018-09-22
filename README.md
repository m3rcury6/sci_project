# sci_project
ros based comparison for computer vision


# some basic notes for the project (for links, see links_help.csv file):
- taking data from formula student team, in order to have leg-up. this is in cooperation with the team.
- will look specifically at blue cones. blue cones have class '1' in the data. ('0' is yellow)
- in *.txt data, first column is class. 2nd & 3rd refer to row (height) percentage, 4 & 5 refer to column (width)
- bounding rectangles seem to be drawn from bottom-left to top-right
- friendly reminder: in opencv, colors are by default BGR (blue-green-red)
- how to pull safely! use the following command: git pull --rebase
- at the outset, it looks like there's only 477 labeled blue cones in the database
- KEEP IN MIND: looks like your *.cfg file plays a big role in how stable or fast your detector is able to run

# progress on darknet side:
- having trouble with tutorial... cuda issue:
- issue with cuda: don't seem to have right architecture or settings. sent a message to stan about this, he mentions that it was not necessary to change any of those settings.
- sep19,15:00 - was able to get darknet running on webcam data. however, may be a long road to testing with cones. recall, this is with
# random helpful commands
- nvidia-smi: outputs basic information about installed nvidia hardware as well as current usage

# note: had to do some simplification on the dataset. removed all photos that didn't contain blue cones, and removed white cone labels. generalsteps: 
  part 1
    1. load an image/text combo
    2. check if it has any blue cones
    3. if yes, close and move on
    4. if no, close files, delete the pair, move on
    5. repeat
  part 2:
    1. for all text files
    2. check if there is any line in a given text file with '0'
    3. if yes, delete that line, then find next, then move on
    4. if no, close file, move on
    5. repeat

# note: yolo training command (as of 2018-sep-22):
cd ~/Desktop/temp_delme/darknet
./darknet detector train data/obj/obj.data data/obj/yolov3-tiny.cfg darknet19_448.conv.23 


# eof
