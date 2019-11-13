# ECE148 Adaptive Cruise Control

ECE 148 FA19 Team 7 Project: adaptive cruise control on autonomous driving RC car.

## Get Started

### Install ROS

We use ROS in this project to work with RPLidar. Therefore please install ROS on your Jetson/Rpi first.

Follow the tutorials below to install ROS and setup catkin workspace:

[http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)<br>
[http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

### Install RPLidar ROS Package

In order to communicate with RPLidar, we use the offical RPLidar ROS package:

[https://github.com/robopeak/rplidar_ros](https://github.com/robopeak/rplidar_ros)

To install this package, ssh into jetson and clone the repo into `catkin_ws/src`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/robopeak/rplidar_ros.git
```

Then build this package:

```bash
cd ~/catkin_ws
catkin_make
```

## Notes

1. Wrong date and time on Jetson

This problem may cause `apt-get update` failure or `catkin_make` failure.

[https://askubuntu.com/questions/929805/timedatectl-ntp-sync-cannot-set-to-yes]
(https://askubuntu.com/questions/929805/timedatectl-ntp-sync-cannot-set-to-yes)

```bash
sudo service ntp stop
sudo ntpd -gq
timedatectl
```
