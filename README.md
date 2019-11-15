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

Wiki page for this package is:

[http://wiki.ros.org/rplidar](http://wiki.ros.org/rplidar)

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

### Wrong date and time on Jetson

This problem may cause `apt-get update` failure or `catkin_make` failure.

[https://askubuntu.com/questions/929805/timedatectl-ntp-sync-cannot-set-to-yes](https://askubuntu.com/questions/929805/timedatectl-ntp-sync-cannot-set-to-yes)

```bash
sudo service ntp stop
sudo ntpd -gq
sudo service ntp start
timedatectl   # check if ntp is synchronized
```

### ROS cannot find package

[https://answers.ros.org/question/190317/ros-cant-find-package/](https://answers.ros.org/question/190317/ros-cant-find-package/)

```bash
cd ~/catkin_ws
source devel/setup.bash
```

### Error when launching RPLidar publisher

When running command `roslaunch rplidar_ros rplidar.launch`, this error message appears:

*Error, cannot bind to the specified serial port /dev/ttyUSB0.*

To solve this, we need to change the permission of ttyUSB0 by:

```bash
sudo chmod 666 /dev/ttyUSB0
```


### Rviz do not show LIDAR scan

After running command ` ` and trying to visualize in Rviz, Rviz does not show LIDAR scan and throws error message: 

*For frame [laser]: Fixed Frame [map] does not exist*

This is because tranformation from world to lidar frame is not defined. You can publish it manually by:

```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 map laser 10
```

Alternatively, you can run this command instead:

```bash
roslaunch rplidar_ros view_rplidar.launch
```
