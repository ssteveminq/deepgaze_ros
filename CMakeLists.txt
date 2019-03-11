cmake_minimum_required(VERSION 2.8.3)
project(deepgaze_ros)

find_package(catkin
    REQUIRED
    COMPONENTS
    rospy
    std_msgs
    message_runtime
    message_generation
    control_msgs
    actionlib_msgs
    actionlib
    pcl_ros
    pcl_conversions
    octomap_ros
    octomap_msgs
    #tmc_yolo2_ros
    villa_yolocloud
    darknet_ros_msgs
    pointcloud_processing_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)

include_directories(${OCTOMAP_INCLUDE_DIRS})
#target_link_libraries(${OCTOMAP_LIBRARIES})

add_service_files(
   FILES
   gaze_recognition_srv.srv
 )

add_action_files(
  DIRECTORY action
  FILES GazeRecog.action
  MultiTrack.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   actionlib_msgs
 )

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

