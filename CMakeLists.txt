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
  geometry_msgs
  nav_msgs
  tf
)

find_package(Boost REQUIRED)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)


add_message_files(FILES BasketAngle.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS cv_bridge genmsg message_generation roscpp rospy sensor_msgs std_msgs geometry_msgs nav_msgs tf
)

add_executable(localization src/odom_broadcaster.cpp)
add_dependencies(localization ${catkin_EXPORTED_TARGETS})
add_dependencies(localization localization_generate_messages_cpp)

target_link_libraries(
    localization
    ${catkin_LIBRARIES}
)
