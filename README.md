# ECE148 Adaptive Cruise Control [![Build Status](https://travis-ci.org/InspireX96/ECE148AdaptiveCruiseControl.svg?branch=master)](https://travis-ci.org/InspireX96/ECE148AdaptiveCruiseControl)

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

## Disclaimer

Travis CI with ROS utilizes `.travis.yml` sections from from [https://github.com/felixduvallet/ros-travis-integration.git](https://github.com/felixduvallet/ros-travis-integration.git)

```
Copyright (c) 2016, Felix Duvallet
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of this package nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

