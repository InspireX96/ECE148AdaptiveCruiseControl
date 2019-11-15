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
from lidar_distance_calculator import calculate_closest_object_distance


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


def player(q, selected_filters):
    """
    Player to draw LIDAR scan animation
    :param q: queue for multi-threading communication
    :param selected_filters: list, list of string indicating selected filters.
                             E.g., selected_filters = ['angular_bounds_filter', 'temporal_median_filter']
    """
    for selected_filter in selected_filters:
        assert selected_filter in ['angular_bounds_filter', 'range_filter', 'temporal_median_filter', 'distance_calculator']

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
    temporal_median_filter = TemporalMedianFilter(k=3)  # initialize temporal median filter

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
        scan_data = {'raw_input': _convert_lidar_msg_to_array(data)}  # place holder for scans

        # NOTE: filters are cascaded
        # apply angular bounds filter
        if 'angular_bounds_filter' in selected_filters:
            scan_data['angular_bounds_filter'] = _convert_lidar_msg_to_array(
                angular_bounds_filter(data, -np.pi / 4, np.pi / 4))

        # apply range filter
        if 'range_filter' in selected_filters:
            scan_data['range_filter'] = _convert_lidar_msg_to_array(range_filter(data, 0, 5))

        # apply temporal median filter
        if 'temporal_median_filter' in selected_filters:
            scan_data['temporal_median_filter'] = _convert_lidar_msg_to_array(temporal_median_filter(data))

        # apply distance calculator
        if 'distance_calculator' in selected_filters:
            scan_data['distance_calculator'] = (np.linspace(0, 2*np.pi, 360), calculate_closest_object_distance(data) * np.ones(360))   # TODO

        # plot
        plt.cla()  # clear plot
        for angles, ranges in scan_data.values():
            plt.polar(angles, ranges)  # plot LIDAR scans

        ax = plt.gca()
        ax.set_rlim(0, 5)  # fix polar plot radius
        plt.legend(scan_data.keys())  # set legend
        plt.title('LIDAR Filter Player')  # set title
        plt.draw()  # update plot
        plt.pause(1e-17)
        q.task_done()


if __name__ == '__main__':
    # init ROS node
    rospy.init_node('scan_listener', anonymous=True)  # ROS node has to be initialized in main thread

    # select filters
    selected_filters = ['angular_bounds_filter', 'range_filter', 'temporal_median_filter', 'distance_calculator']     # TODO: select filter

    q = queue.Queue()
    # spin two threads to receive LIDAR topic and draw animation simultaneously
    threads = [threading.Thread(target=listener, args=(q,)),
               threading.Thread(target=player, args=(q, selected_filters))]
    for t in threads:
        t.start()

    print('Please hit Ctrl+C to quit LIDAR filter player')
    q.join()
    for t in threads:
        t.join()
