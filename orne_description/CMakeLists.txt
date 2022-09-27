cmake_minimum_required(VERSION 2.8.3)
project(orne_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  xacro
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orne_description
#  CATKIN_DEPENDS icart_mini_description xacro
#  DEPENDS system_lib
)
install(DIRECTORY rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
