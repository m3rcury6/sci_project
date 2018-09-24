/*
Authors: Kris Gonzalez, Nitin Nayak
Objective: have simple c++ talker / listener node

How to run (after compiling with catkin_make):
>> roslaunch ai_darknet ai_darknet.launch

Task List:
STATUS | DESCRIPTION
=======|========================

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
//libraries included in Detector.cpp (from fstw)
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//KJG_Sep24: was originally "CDarknet.h", but changing to "darknet.h" worked...
#include "darknet.h" //requires change in CMakeLists.txt file

#include <ros/console.h>







void chatter_Callback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]",msg->data.c_str());
} //void chatter_Callback

int main(int argc, char **argv){ //argc / argv enable input arguments

  //input argument review
  std::cout << "Roslaunch arguments ("<< argc-3 <<"):\n";
  for(int i=1;i<argc-2;i++){
    std::cout << argv[i]<<"\n";
  }//forloop
  std::cout<<"\n";

  // initialize node, etc.
  ros::init(argc,argv,"ai_darknet_node");
  ros::NodeHandle n;
  ros::Publisher pub_str = n.advertise<std_msgs::String>("chatter",1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while(ros::ok() && count < 5){
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s",msg.data.c_str());
    pub_str.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }//while loop
  return 0;
}//int main
