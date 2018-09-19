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


# eof
