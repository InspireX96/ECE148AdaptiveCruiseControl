# ECE148 Adaptive Cruise Control [![Build Status](https://travis-ci.org/InspireX96/ECE148AdaptiveCruiseControl.svg?branch=master)](https://travis-ci.org/InspireX96/ECE148AdaptiveCruiseControl)

ECE/MAE 148 FA19 Team 7 Project: adaptive cruise control on autonomous driving RC car.

Please checkout our wiki page for more information: [https://guitar.ucsd.edu/maeece148/index.php/2019FallTeam7#References](https://guitar.ucsd.edu/maeece148/index.php/2019FallTeam7#References)

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

### Install YOLO

This project uses YOLO for pedestrain and traffic sign detection. Please install YOLO under `projects/d3` following the instruction:

[https://pjreddie.com/darknet/install/](https://pjreddie.com/darknet/install/)

Make sure to download pretrained YOLO weights. For more information please refer to:

[https://pjreddie.com/darknet/yolo/](https://pjreddie.com/darknet/yolo/)

## Run the Codes

### Clone this repo
```bash
cd ~/projects
git clone https://github.com/InspireX96/ECE148AdaptiveCruiseControl.git
```

### Install required python packages

Install required python packages in addition to packages in donkeycar framework

```bash
cd ECE148AdaptiveCruiseControl
pip install -r requirements.txt   # NOTE: you may need to run this command in sudo
```

### Setup

To setup this project on top of existing donkey framework, simply run command:

```bash
cd src
./setup.sh
```

### Turn on RPLidar

Use the launch file in RPLidar ROS package to turn on the LIDAR.

```bash
roslaunch rplidar_ros rplidar.launch
```

If you get some error messages here, please refer to the **Notes** section

### Run the vehicle

Now it's time to run the vehicle!

```bash
cd ~/projects/d3
python manage_modified.py drive
```

### Autopilot

Autopilot feature is preserved in `manage_modified.py`. Usage is the same as the original `manage.py`. Please refer to the original donkey framework for more information.

#### Different drive modes

There are three drive modes to select:

1. Manual steering and throttle
2. Manual steering and adaptive cruise control throttle
3. AI steering and adaptive cruise control throttle

## LIDAR Filter Player App

To visualize different LIDAR filters and LIDAR distance calculaor, you can run `lidar_filter_player.py`.

Before running this script, please make sure ROS is publishing LIDAR scans, so you can visualize them in real time.


## Tests

You can manually run unit tests of this package. Please install the `setup.py` in order to let *pytest* find this package on your computer. You can setup using *pip*:

```bash
pip install -e .
```

Then run the unit tests:

```bash
pytest -v
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

After running command `roslaunch rplidar_ros rplidar.launch` and trying to visualize in Rviz, Rviz does not show LIDAR scan and throws error message: 

*For frame [laser]: Fixed Frame [map] does not exist*

This is because tranformation from world to lidar frame is not defined. You can publish it manually by:

```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 map laser 10
```

Alternatively, you can run this command instead:

```bash
roslaunch rplidar_ros view_rplidar.launch
```

### Python3 cannot import rospkg

```
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
```

## Reference

This project is intended to be used within the donkey car framework [https://github.com/autorope/donkeycar.git](https://github.com/autorope/donkeycar.git)

Object detection utilizes YOLO:

```
@article{yolov3,
  title={YOLOv3: An Incremental Improvement},
  author={Redmon, Joseph and Farhadi, Ali},
  journal = {arXiv},
  year={2018}
}
```

Travis CI with ROS utilizes `.travis.yml` sections from from [https://github.com/felixduvallet/ros-travis-integration.git](https://github.com/felixduvallet/ros-travis-integration.git)

```
Copyright (c) 2016, Felix Duvallet
All rights reserved.
```

