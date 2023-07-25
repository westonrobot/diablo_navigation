# Diablo ROS Workspace


## About
#
This software package provides a ROS1 C++ functionality to implement SLAM and Navigation algorithms using Direct Drive Technology's robot called "Diablo"
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
$ git clone https://github.com/DDTRobot/diablo_sdk.git
$ git clone https://github.com/Slamtec/rplidar_ros.git
```
Ensure to also install the dependencies of these repositories:
- diablo_sdk (https://github.com/DDTRobot/diablo_sdk.git)
- rplidar_ros (https://github.com/Slamtec/rplidar_ros.git)

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
Once you see that the map is complete on RVIZ, save the map in diablo_base/maps
```bash
$ mapserver mapsaver
```
<br> 
</br>

### Perform navigation
```bash
$ roslaunch diablo_2dnav setup_2dnav.launch
$ roslaunch diablo_2dnav start_2dnav.launch
$ rosrun rviz rviz
```
Current Pose Estimate and 2D Nav Goal can be set through the RVIZ interface to test navigation