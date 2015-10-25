orne_navigation
=================

[![Join the chat at https://gitter.im/open-rdc/orne_navigation](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/open-rdc/orne_navigation?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This repository provide basic navigation system for Tsukuba Challenge using i-Cart mini by Project ORNE.

[![Throughput Graph](https://graphs.waffle.io/open-rdc/tsukubachallenge/throughput.svg)](https://waffle.io/open-rdc/tsukubachallenge/metrics) 

## Dependency Repositories

* https://github.com/open-rdc/icart_mini

* https://github.com/open-rdc/cit_adis_imu

* https://github.com/open-rdc/orne_maps

* https://github.com/open-rdc/orne_icart_designs

## Install

Install ROS software (recommended ROS indigo version with Ubuntu 14.04LTS) at http://www.ros.org/wiki/ROS/Installation, please select Ubuntu platform.

```sh
$ cd CATKIN_WORKSPACE/src
$ wstool init
$ wstool merge https://raw.githubusercontent.com/open-rdc/orne_navigation/indigo-devel/orne_pkgs.install
$ wstool up
$ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
$ cd ..
$ catkin_make
```

## Usage

### Bring up the real/simulated robot

The following will show the commands needed to bring up either real or simulated robots.

* Bring up the simulated robot

```sh
$ roslaunch orne_bringup orne_alpha_sim.launch // or orne_beta_sim.launch
```

* Bring up the real robot

```sh
$ roslaunch orne_bringup orne_alpha.launch // or orne_beta.launch
```

### Build map

```sh
$ roslaunch orne_navigation build_map_teleop.launch
```

During building a map, waypoints are recorded by pressing the No.1 button of the joystick.

When you set 2DNavGoal at the goal point on the RViz, waypoints will be saved and then waypoints file stored in orne_navigation/waypoints_cfg/waypoints.yaml is overwritten. So that, the navigation system can be automatically load waypoints configuration.

If you want to save a map, run a map_saver node like the following command.

```sh
$ rosrun map_server map_saver -f filename
```

### Record the waypoints

* Using the PublishPoint message on the RViz

```sh
$ roslaunch orne_navigation record_waypoints_viz.launch map_file:=filename.yaml
```

* Using a joystick

```sh
$ roslaunch orne_navigation record_waypoints_joy.launch map_file:=filename.yaml
```

Note that filename must be specified in the full path.

### Navigation

* Waypoint Navigation

```sh
$ roslaunch orne_navigation play_waypoints_nav.launch
```

* Waypoint Navigation with an optional map file

```sh
$ roslaunch orne_navigation play_waypoints_nav.launch map_file:=filename.yaml
```

A map name must be specified in the full path.

* Run the navigation system with a static map

```sh
$ roslaunch orne_navigation nav_static_map.launch
```

* Enable the starting flag

```sh
$ rostopic pub -1 /syscommand std_msgs/String "start"
```

Don't forget to turn off the teleoperation, it might interfere with the robot's commands.

## Bugs & Tasks

https://github.com/open-rdc/orne_navigation/issues

## License

License-check is open source software under the [BSD license](https://github.com/open-rdc/icart_mini_ros_pkgs/blob/master/LICENSE).
