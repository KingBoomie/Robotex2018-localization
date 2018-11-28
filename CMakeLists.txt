cmake_minimum_required(VERSION 2.8.3)
project(localization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  genmsg
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  serial
)

catkin_package(
  CATKIN_DEPENDS cv_bridge genmsg message_generation roscpp rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_message_files(FILES BasketAngle.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()
