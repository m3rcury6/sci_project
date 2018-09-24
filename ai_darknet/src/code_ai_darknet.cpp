/*
Authors: Kris Gonzalez, Nitin Nayak
Objective: have simple c++ talker / listener node

How to run (after compiling with catkin_make):
>> roslaunch ai_darknet ai_darknet.launch

Task List:
STATUS | DESCRIPTION
=======|========================


General Steps of program:
1. intialize ros node and all functions
2. listen for incoming image
3. analyze image for cones
4. return in string the name and number of cones detected
5. repeat
*/

// 0.0 initializations =========================================================
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
//libraries included in Detector.cpp (from fstw)
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>

//KJG_Sep24: was originally "CDarknet.h", but changing to "darknet.h" worked...
#include "darknet.h" //requires change in CMakeLists.txt file


void chatter_Callback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]",msg->data.c_str());
} //void chatter_Callback

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
		ROS_INFO("I heard something...\n");
		// pDarknet->SetInput(cv_bridge::toCvShare(msg, "rgb8")->image);
    // ConeDetection::bbox_array oMessage = pDarknet->Forward();
    // oMessage.header.stamp = msg->header.stamp;
    // oDetectionPublisher.publish(oMessage);
		//
    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", pDarknet->GetDetections()).toImageMsg();
		//
    // pub2.publish(msg);

    ros::spinOnce(); //kjgnote: is this necessary here?

   // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
 }//try
  catch (cv_bridge::Exception& e){
		ROS_ERROR("kjgnote: somehow ended up here while debugging...");
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }//catch
}//void imageCallback



int main(int argc, char **argv){ //argc / argv enable input arguments

  //input argument review
  std::cout << "Roslaunch arguments ("<< argc-3 <<"):\n";
  for(int i=1;i<argc-2;i++){
    std::cout << argv[i]<<"\n";
  }//forloop
  std::cout<<"\n";

	// initialize some files and pointers
	char* pDATA   = argv[1];
	char* pCFG    = argv[2];
	char* pWEIGHT = argv[3];
	char* sub_name= argv[4];
	char* sub_img = argv[5];
	char* pub_res = argv[6];


  // initialize node, etc.
  ros::init(argc,argv,"ai_darknet_node");
  ros::NodeHandle nh;
  ros::Publisher pub_str = nh.advertise<std_msgs::String>("detector_result",10);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/images",1,imageCallback);





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
