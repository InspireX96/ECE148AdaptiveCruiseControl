"""
A player to visualize LIDAR filters
"""
import time
import queue
import threading
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np
from matplotlib import pyplot as plt


def callback(data, args):
    """
    Call back function of ROS subscriber
    :param data: LaserScan msg, LIDAR scan data
    :param args: arguments. arg[0] is queue
    """
    q = args[0]
    q.put(data)     # put LIDAR msg in queue and get another thread to plot?


def listener(q):
    """
    Listener to receive LIDAR scans
    :param q: queue for multi-threading communication
    """
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('scan_listener', anonymous=True)  # init ROS node

    rospy.Subscriber('scan', LaserScan, callback, (q, ))  # subscribe

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def player(q):
    """
    Player to draw LIDAR scan animation
    :param q: queue for multi-threading communication
    :return:
    """
    def _convert_lidar_msg_to_array(data):
        """
        Convert LIDAR ROS msg to np array
        :param data: LaserScan msg, LIDAR scan data
        :returns: np array, angles and ranges
        """
        angle_min, angle_max, ranges = data.angle_min, data.angle_max, data.ranges
        angles = np.linspace(angle_min, angle_max, len(ranges))
        ranges = np.array(ranges)
        return angles, ranges

    plt.show()      # init figure

    while True:
        # get LIDAR data
        try:
            data = q.get(timeout=10)
        except Exception as err:
            print('Timeout receiving LIDAR data: {}\n'
                  'This thread will stop automatically,\n'
                  '  Please hit Ctrl+C to stop other threads'.format(err))
            return

        # parse LIDAR data
        angles, ranges = _convert_lidar_msg_to_array(data)

        plt.cla()
        plt.polar(angles, ranges, 'r')
        plt.polar(0.8*angles, 0.8*ranges, 'b')
        plt.draw()
        plt.pause(1e-17)
        q.task_done()


if __name__ == '__main__':
    # init ROS node
    rospy.init_node('scan_listener', anonymous=True)  # ROS node has to be initialized in main thread

    q = queue.Queue()
    # spin two threads to receive LIDAR topic and draw animation simultaneously
    threads = [threading.Thread(target=listener, args=(q, )),
               threading.Thread(target=player, args=(q, ))]
    for t in threads:
        t.start()

    print('Please hit Ctrl+C to quit LIDAR player')
    q.join()
    for t in threads:
        t.join()

