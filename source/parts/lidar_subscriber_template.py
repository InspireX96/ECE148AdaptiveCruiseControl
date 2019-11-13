#!/usr/bin/env python
"""
A template to subscribe LIDAR scans from ROS
"""

import rospy
from sensor_msgs.msg import LaserScan


def callback(data):
    """
    Call back function of ROS subscriber
    :param data: LaserScan msg, LIDAR scan data
    """
    print(data)


def listener():
    """
    Listener to receive LIDAR scans
    """
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('scan_listener', anonymous=True)  # init ROS node

    rospy.Subscriber('scan', LaserScan, callback)  # subscribe

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
