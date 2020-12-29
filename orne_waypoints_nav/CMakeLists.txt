cmake_minimum_required(VERSION 2.8.3)
project(orne_waypoints_nav)

find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  geometry_msgs
  move_base
  roscpp
  actionlib_msgs
  actionlib
  tf
  std_srvs
  message_generation
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

## Generate services in the 'srv' folder
add_service_files(
    FILES
    Pose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  #include 
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

add_executable(waypoints_saver src/waypoints_saver.cpp)
add_executable(waypoints_nav src/waypoints_nav.cpp)

add_dependencies(waypoints_nav ${orne_waypoints_nav_EXPORTED_TARGETS})

target_link_libraries(waypoints_saver
    ${catkin_LIBRARIES}
)

target_link_libraries(waypoints_nav
    ${catkin_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)

