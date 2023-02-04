cmake_minimum_required(VERSION 2.8.3)
project(orne_stoppoint_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  std_msgs
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  tf
  LIBRARIES ${PROJECT_NAME}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)
add_executable(orne_stoppoint_sender src/stoppoint_sender.cpp)

target_link_libraries(orne_stoppoint_sender
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)
