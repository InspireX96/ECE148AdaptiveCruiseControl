"""
A player to visualize LIDAR filters
Please have LIDAR scan publisher ready before running this script
"""

import copy
import queue
import threading
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np
from matplotlib import pyplot as plt

from lidar_filters import angular_bounds_filter, range_filter, TemporalMedianFilter


def callback(data, args):
    """
    Call back function of ROS subscriber
    :param data: LaserScan msg, LIDAR scan data
    :param args: arguments. arg[0] is queue
    """
    q = args[0]
    q.put(data)  # put LIDAR msg in queue and get another thread to plot?


def listener(q):
    """
    Listener to receive LIDAR scans
    :param q: queue for multi-threading communication
    """
    rospy.Subscriber('scan', LaserScan, callback, (q,))  # subscribe

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def player(q):
    """
    Player to draw LIDAR scan animation
    :param q: queue for multi-threading communication
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

    plt.show()  # initialize figure

    while True:
        # get LIDAR data
        try:
            data = q.get(timeout=5)
        except Exception as err:
            print('Timeout receiving LIDAR data: {}\n'
                  'This thread will stop automatically,\n'
                  '  Please hit Ctrl+C to stop other threads'.format(err))
            return

        # parse LIDAR data
        raw_angles, raw_ranges = _convert_lidar_msg_to_array(data)

        # apply angular bounds filter
        angular_bounds_angles, angular_bounds_ranges = _convert_lidar_msg_to_array(
            angular_bounds_filter(data, -np.pi / 4, np.pi / 4))

        # plot
        plt.cla()  # clear plot
        plt.polar(raw_angles, raw_ranges, 'r')
        plt.polar(angular_bounds_angles, angular_bounds_ranges, 'b')

        ax = plt.gca()
        ax.set_rlim(0, 5)  # fix polar plot radius
        plt.legend(['1', '2'])  # set legend
        plt.title('LIDAR Filter Player')  # set title
        plt.draw()  # update plot
        plt.pause(1e-17)
        q.task_done()


if __name__ == '__main__':
    # init ROS node
    rospy.init_node('scan_listener', anonymous=True)  # ROS node has to be initialized in main thread

    q = queue.Queue()
    # spin two threads to receive LIDAR topic and draw animation simultaneously
    threads = [threading.Thread(target=listener, args=(q,)),
               threading.Thread(target=player, args=(q,))]
    for t in threads:
        t.start()

    print('Please hit Ctrl+C to quit LIDAR filter player')
    q.join()
    for t in threads:
        t.join()
