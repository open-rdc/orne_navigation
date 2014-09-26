t_frog_ros_pkgs
=================
This repository contains metapackages and files for installation/use of the T-frog.

## Other T-frog Repositories

Package | Repository URL
------- | --------------
t_frog_slam | https://github.com/open-rdc/t_frog_slam
t_frog_driver | https://github.com/open-rdc/t_frog_driver
t_frog_navigation | https://github.com/open-rdc/t_frog_navigation
t_frog_gazebo | https://github.com/open-rdc/t_frog_gazebo

## Install

Install ROS software (recommended ROS indigo version with Ubuntu 14.04LTS) at http://www.ros.org/wiki/ROS/Installation, please select Ubuntu platform. 

```sh
$ cd CATKIN_WORKSPACE/src
$ wstool init
$ git clone https://github.com/open-rdc/t_frog_ros_pkgs
$ wstool merge t_frog_ros_pkgs/t_frog_ros_pkgs.install
$ wstool update
$ cd ..
$ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
$ catkin_make
```

## Usage

#### Setup

```sh
$ ./t-frog.sh
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

If you find a bug please let me know by opening an issue at: https://github.com/open-rdc/t_frog_ros_pkgs/issues

## License

License-check is open source software under the [MIT license](https://github.com/open-rdc/t_frog_ros_pkgs/blob/master/LICENSE).
