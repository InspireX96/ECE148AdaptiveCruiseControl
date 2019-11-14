"""
A player to visualize LIDAR filters
"""
import random
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
    ysample = random.sample(range(-50, 50), 100)

    xdata = []
    ydata = []

    plt.show()

    axes = plt.gca()
    axes.set_xlim(0, 100)
    axes.set_ylim(-50, +50)
    line, = axes.plot(xdata, ydata, 'r-')

    for i in range(100):
        xdata.append(i)
        ydata.append(ysample[i])
        line.set_xdata(xdata)
        line.set_ydata(ydata)
        plt.draw()
        plt.pause(1e-17)
        time.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('scan_listener', anonymous=True)  # init ROS node
    # listener()
    t1 = threading.Thread(target=listener)
    t2 = threading.Thread(target=player)
    t1.start()
    t2.start()
    t1.join()
    t2.join()

