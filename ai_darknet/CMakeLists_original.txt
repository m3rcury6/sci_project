cmake_minimum_required(VERSION 2.8.3)
project(ai_darknet)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  # pcl_conversions
  # pcl_ros
  std_msgs
  genmsg
  message_generation
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV 3 REQUIRED)

include_directories(
  ./darknet_lib/include
  ./darknet_lib/src
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY msg
  FILES
  bbox.msg
  bbox_array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    # pcl_conversions
    # pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)


add_executable(ai_darknet src/code_ai_darknet.cpp src/CDarknet.cpp src/CDarknet.h)
target_link_libraries(ai_darknet ${catkin_LIBRARIES} /usr/local/lib/libdarknet.so)
add_dependencies(ai_darknet ai_darknet_generate_messages_cpp)
# target_link_libraries(Detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} /usr/local/lib/libdarknet.so) # MAY NEED THIS
