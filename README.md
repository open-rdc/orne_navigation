icart_mini_ros_pkgs
=================
This repository contains metapackages and files for installation/use of the i-Cart mini.

## Other i-Cart mini Repositories

Package | Repository URL
------- | --------------
icart_mini_core | https://github.com/open-rdc/icart_mini_core
icart_mini_navigation | https://github.com/open-rdc/icart_mini_navigation
icart_mini_gazebo | https://github.com/open-rdc/icart_mini_gazebo

## Install

Install ROS software (recommended ROS indigo version with Ubuntu 14.04LTS) at http://www.ros.org/wiki/ROS/Installation, please select Ubuntu platform. 

```sh
$ cd CATKIN_WORKSPACE/src
$ wstool init
$ git clone https://github.com/open-rdc/icart_mini_ros_pkgs
$ wstool merge icart_mini_ros_pkgs/icart_mini_ros_pkgs.install
$ wstool up
$ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
$ cd ..
$ catkin_make
```

## Usage

#### Bring up the real/simulated robot

The following will show the commands needed to bring up either real or simulated robots.

 * Bring up the simulated robot

```sh
$ roslaunch icart_mini_gazebo icart_mini.launch
```

 * Bring up the real robot

```sh
$ ./icart-mini.sh
$ roslaunch icart_mini_driver icart_mini_drive.launch
```

#### Build map

```sh
$ roslaunch icart_mini_navigation build_map_teleop.launch
```

#### Record the waypoints

 * Using the publish point on RViz

```sh
$ roslaunch icart_mini_navigation record_waypoints_viz.launch
```

 * Using the Joystick

```sh
$ roslaunch icart_mini_navigation record_waypoints_joy.launch
```

#### Navigation

 * Waypoint Navigation

```sh
$ roslaunch icart_mini_navigation play_waypoints_nav.launch
```

 * Path Planning and Navigation in static map

```sh
$ roslaunch icart_mini_navigation nav_static_map.launch
```

## Bugs & Tasks

https://github.com/open-rdc/icart_mini_ros_pkgs/issues

## License

License-check is open source software under the [BSD license](https://github.com/open-rdc/icart_mini_ros_pkgs/blob/master/LICENSE).
