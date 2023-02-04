cmake_minimum_required(VERSION 2.8.3)
project(orne_recovery_behavior)

set(CMAKE_CXX_FLAGS "-march=native -O2 -std=c++11 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS   "-march=native -O2 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-march=native -O2 -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "-march=native -O2 -fopenmp")

# check c++14 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  move_base
  std_msgs
  geometry_msgs
  sensor_msgs
  laser_geometry
  tf
  pcl_ros
  pcl_conversions
  pcl_msgs
  costmap_2d
  pluginlib
  nav_core
  base_local_planner
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
      roscpp
      pcl_ros
      pcl_conversions
      pcl_msgs
      pluginlib
)

include_directories(
  #include 
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  src
)

add_library(simple_lead_out src/simple_lead_out.cpp)
target_link_libraries(simple_lead_out ${catkin_LIBRARIES})
