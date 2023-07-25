# Diablo ROS Workspace

This repository provides a ROS1 navigation setup for the two leg-wheel [Diablo robot](https://docs.westonrobot.net/robot_user_guide/direct_drive_robot/diablo.html). Please note that this setup is mainly for demonstration purpose and you may need to make adjustments and fine tuning to use it for your application.

## Build and Install

### Install the dependencies

Create a catkin workspace if you don't have one yet:

```bash
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
```

Clone the driver packages for the robot and Lidar. Please ensure you've also installed the dependencies of them. You can refer to their documentations for more details. 

```bash
$ git clone https://github.com/westonrobot/diablo_sdk.git
$ git clone https://github.com/westonrobot/rplidar_ros
```

Install the ROS navigation packages:

```bash
$ sudo apt install ros-$ROS_DISTRO-navigation
$ sudo apt install ros-$ROS_DISTRO-gmapping
$ sudo apt install ros-$ROS_DISTRO-amcl
```

### Build the workspace
```bash
$ cd catkin_ws
$ catkin_make
```

## Example usage

### Mapping

Launch the gmapping nodes:

```bash
$ roslaunch diablo_navigation start_mapping.launch
$ rosrun rviz rviz
```

Once you see that the map is complete on RVIZ, save the map in diablo_base/maps
```bash
$ rosrun map_server map_saver -f <map_name>
```

### Navigation
```bash
$ roslaunch diablo_navigation bringup_robot.launch
$ roslaunch diablo_navigation start_navigation.launch
$ rosrun rviz rviz
```

Current Pose Estimate and 2D Nav Goal can be set through the RVIZ interface to test navigation