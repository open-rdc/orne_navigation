cmake_minimum_required(VERSION 2.8.3)
project(stoppoint_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  tf
)

add_executable(stoppoint_sender src/stoppoint_sender.cpp)

target_link_libraries(stoppoint_sender
    ${catkin_LIBRARIES}
)
