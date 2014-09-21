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

```sh
$ cd CATKIN_WORKSPACE/src
$ wstool init
$ wstool merge t_frog_ros_pkgs.rosinstall
$ wstool update
$ cd ..
$ rosdep install --from-paths . --ignore-src --rosdistro `$ROS_DISTRO` -y
$ catkin_make
```

## License

License-check is open source software under the [MIT license](https://github.com/open-rdc/t_frog_ros_pkgs/blob/master/LICENSE).
