"""
Calculate closed object distance from LIDAR scans
"""
import logging
import numpy as np
import matplotlib.pyplot as plt


def calculate_closest_object_distance(scan, debug=False):
    """
    This function will apply a histogram filter on LIDAR scan to remove possible noise,
    then calculated the object distance.

    :param scan: LaserScan msg, LIDAR scan data
    :param debug: bool, flag to show debug plots
    :return: float, closest object distance
    """
    ranges = np.array(scan.ranges)

    # histogram filter
    hist = plt.hist(ranges, bins='auto')
    distance = np.mean(ranges[np.logical_and(ranges >= hist[1][0], ranges <= hist[1][1])])

    if debug:
        plt.plot([distance, distance], [0, np.max(hist[0])])
        plt.title('Calculated Distance: {}'.format(distance))
        plt.waitforbuttonpress(1)
        plt.draw()
        plt.close()

    logging.info('hist: {}, distance: {}'.format(hist, distance))
    return distance
