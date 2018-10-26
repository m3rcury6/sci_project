# sci_project
ROS-based comparison for computer vision algorithms

## Authors
Kristian Gonzalez, Masters Mechatronics, Hochschule Ravensburg Weingarten
Nitin Nayak, Masters Mechatronics, Hochschule Ravensburg Weingarten

## Objective
We built this repository as part of our scientific project work at HRW. The repository serves to develop and test multiple object detection algorithms under the same conditions. 

NOTE: this readme file only serves to summarize basic information about the repository. For the full report, please see our final report (link coming soon). 

#### Basic Project Side Notes
- taking data from formula student team, in order to have quick start. this is in cooperation with the team.
- will look specifically at blue cones. blue cones have class '1' in the data. ('0' is yellow)
- in *.txt data, column data is comprised of (class, xcp, ycp, hp, wp). 'c' refers to center, and 'p' refers to percentage, e.g. x_center location as a percentage of total image width.
- friendly reminder: in opencv, colors are by default BGR (blue-green-red)
- how to pull safely! use the following command: git pull --rebase
- Simplification of FSTW dataset: Had to do some simplification on the dataset: removed all photos that didn't contain blue cones, and removed yellow cone labels.
- Example functioning darknet training command: ./darknet detector train data/obj/obj.data data/obj/yolov3-tiny.cfg darknet19_448.conv.23
