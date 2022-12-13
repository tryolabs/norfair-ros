cmake_minimum_required(VERSION 3.0.2)
project(norfair_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
    Detection.msg 
    Detections.msg
    Point.msg
    BoundingBox.msg
    BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
