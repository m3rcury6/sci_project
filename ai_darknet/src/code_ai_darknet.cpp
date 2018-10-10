/*
Authors: Kris Gonzalez, Nitin Nayak
Objective: have simple c++ talker / listener node

How to run (after compiling with catkin_make):
>> roslaunch ai_darknet ai_darknet.launch

Task List:
STATUS | DESCRIPTION
=======|========================


General Steps of program:
1. intialize ros node and all functions done
2. listen for incoming image
3. analyze image for cones
4. return in string the name and number of cones detected
5. repeat
*/

// 0.0 initializations =========================================================
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "CDarknet.h" //requires change in CMakeLists.txt file
#include <ros/console.h>

#include "std_msgs/String.h"
#include <sstream>
#include <string> //kjgnote: may not be necessary
#include <iostream>

// just including to have a random number generator (for fun)
#include <math.h>
#include <cstdlib>
#include <ctime>



CDarknet* pDarknet = nullptr;



ros::Publisher pub_res; // need to declare here to use in functions
ros::Publisher pub_bboxes; // need to declare here to use in functions
std::string IMG_NAME ("empty");
//char* IMG_NAME;

void call_name(const std_msgs::String::ConstPtr& msg){
  IMG_NAME = msg->data.c_str();
//  ROS_INFO("call_name: %s",msg->data.c_str());
//  std::cout << "in string:"<<IMG_NAME<<"\n";
}// void call_name

void call_image(const sensor_msgs::ImageConstPtr& imgmsg){
  try{
    ROS_INFO("recieved: %s",IMG_NAME.c_str());

    // here, will test an algorithm

    pDarknet->SetInput(cv_bridge::toCvShare(imgmsg, "rgb8")->image);
    ai_darknet::bbox_array oMessage = pDarknet->Forward();

    std::cout << "estimated number of cones: " << oMessage.bboxes.size() << '\n';
    // kjgnote: oMessage.bboxes is basically a vector object
    // likely now have a message with a specific number of boxes. now, want to
    //   simply count them then publish that number.

    oMessage.header.stamp = imgmsg->header.stamp;
    oMessage.header.frame_id = IMG_NAME.c_str();
    pub_bboxes.publish(oMessage); //publish actual bounding boxes of img
    // oDetectionPublisher.publish(oMessage);
    //
    // KJGNOTE: this below section was to publish a new image. won't do this
    // sensor_msgs::ImagePtr newmsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", pDarknet->GetDetections()).toImageMsg();
    //
    // pub2.publish(newmsg);


    // once complete, algorithm returns an integer value

    // float rvar = std::rand();
    // rvar = rvar/RAND_MAX*10;
    int res = oMessage.bboxes.size();
    // int res = 4; //this would be the result after img processing has happened
		IMG_NAME = IMG_NAME + ","+std::to_string(res);
		std_msgs::String msg_res;
		msg_res.data = IMG_NAME;
		pub_res.publish(msg_res);
		ros::spinOnce(); //kjgnote: is this necessary here?

   // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
 }//try
  catch (cv_bridge::Exception& e){
		ROS_ERROR("kjgnote: somehow ended up here while debugging...");
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgmsg->encoding.c_str());
  }//catch
}//void call_image


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
	char* arg_subname = argv[4];
	char* arg_subimg  = argv[5];
	char* arg_pubres  = argv[6];




  // initialize node, etc.
  const char* nodename = "ai_darknet_node"; // remember to use const char* for string constant
  ros::init(argc,argv,nodename);
  ros::NodeHandle nh;

  // setup new darknet neural network object
  pDarknet = new CDarknet(pDATA,pCFG,pWEIGHT); // initialize
  pDarknet->SetNClasses(1); // have only one class to identify

  // setup subscriber for image
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe(arg_subimg,1,call_image);

  // setup subscriber for imgname
  ros::Subscriber sub_name = nh.subscribe(arg_subname,1,call_name);

  // setup publisher for result
  pub_res = nh.advertise<std_msgs::String>(arg_pubres,10);

  //setup publisher for bboxes
  pub_bboxes = nh.advertise<ai_darknet::bbox_array>("bboxes",1);


  ros::Rate loop_rate(10);
  int count = 0;

  std::cout<< nodename << " started.\n";
  while(ros::ok()){
    ros::spinOnce();
  }//whileloop


//  while(ros::ok() && count < 5){
//    std_msgs::String msg;
//    std::stringstream ss;
//    ss << "imgname," << count;
//    msg.data = ss.str();
//    ROS_INFO("%s",msg.data.c_str());
////    pub_str.publish(msg); // don't publish just yet
//    ros::spinOnce();
//    loop_rate.sleep();
//    ++count;
//  }//while loop
  return 0;
}//int main
