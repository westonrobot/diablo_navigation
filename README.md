# Diablo ROS Workspace


## About
#
This software package provides a ROS1 c++ functionality to implement SLAM and Navigation algorithms using Direct Drive Technology's robot called "Diablo"
<br> 
</br>

## Build and Install
#

### Install the dependencies
```bash
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
```

Clone the following repositories:
```bash
$ git clone //diablo_sdk
$ git clone //rplidar_ros
```
Ensure to install its own dependencies:
- diablo_sdk ( )
- rplidar_ros ( )

Install other dependencies:
```bash
$ sudo apt install ros-<DISTRO>-navigation
$ sudo apt install ros-<DISTRO>-gmapping
$ sudo apt install ros-<DISTRO>-amcl
```

### Build the workspace
```bash
$ cd catkin_ws
$ catkin_make
```
<br> 
</br>

## Example usage
#

### Generate map
```bash
$ roslaunch diablo_base slam_gmapping.launch
$ rosrun rviz rviz
```

### Perform navigation
```bash
$ roslaunch diablo_2dnav setup_2dnav.launch
$ roslaunch diablo_2dnav start_2dnav.launch
$ rosrun rviz rviz
```

