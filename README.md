icart_mini_ros_pkgs
=================
This repository contains metapackages and files for installation/use of the i-Cart mini.

## Other i-Cart mini Repositories

Package | Repository URL
------- | --------------
icart_mini_driver | https://github.com/open-rdc/icart_mini_driver
icart_mini_navigation | https://github.com/open-rdc/icart_mini_navigation
icart_mini_gazebo | https://github.com/open-rdc/icart_mini_gazebo

## Install

Install ROS software (recommended ROS indigo version with Ubuntu 14.04LTS) at http://www.ros.org/wiki/ROS/Installation, please select Ubuntu platform. 

```sh
$ cd CATKIN_WORKSPACE/src
$ wstool init
$ git clone https://github.com/open-rdc/icart_mini_ros_pkgs
$ wstool merge icart_mini_ros_pkgs/icart_mini_ros_pkgs.install
$ wstool update
$ cd ..
$ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
$ catkin_make
```

## Usage

#### Setup

 * running simulator

```sh
$ roslaunch icart_mini_gazebo icart_mini.launch
```

 * start-up real robot

```sh
$ ./icart-mini.sh
$ roslaunch icart_mini_driver icart_mini_drive.launch
```

#### Build map

```sh
$ roslaunch t_frog_slam build_map_teleop.launch
```

#### Navigation

```sh
$ roslanch t_frog_navigation nav_static_map.launch
```

## Bugs

If you find a bug please let me know by opening an issue at: https://github.com/open-rdc/icart_mini_ros_pkgs/issues

## License

License-check is open source software under the [MIT license](https://github.com/open-rdc/icart_mini_ros_pkgs/blob/master/LICENSE).
