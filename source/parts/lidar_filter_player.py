"""
A player to visualize LIDAR filters
"""
import time
import threading
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np
from matplotlib import pyplot as plt


def callback(data, args):
    """
    Call back function of ROS subscriber
    :param data: LaserScan msg, LIDAR scan data
    """
    # TODO: put stuff in queue and get another thread to plot?
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
    # rospy.init_node('scan_listener', anonymous=True)  # init ROS node

    rospy.Subscriber('scan', LaserScan, callback, (None, ))  # subscribe

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def player():
    plt.show()

    for i in range(100):
        plt.cla()
        plt.scatter(np.random.rand(10), np.random.rand(10))
        plt.draw()
        plt.pause(1e-17)


if __name__ == '__main__':
    # int ROS node
    rospy.init_node('scan_listener', anonymous=True)  # ROS node has to be initialized in main thread

    # spin two threads to receive LIDAR topic and draw animation simultaneously
    threads = [threading.Thread(target=listener), threading.Thread(target=player)]
    for t in threads:
        t.start()

    for t in threads:
        t.join()

