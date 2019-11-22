"""
Calculate closed object distance from LIDAR scans
"""
import numpy as np
import matplotlib.pyplot as plt  # TODO: remove matplotlib, put it in tests


def calculate_closest_object_distance(scan, debug=False):
    """
    This function calculates object distance from given LIDAR scan.

    :param scan: LaserScan msg, LIDAR scan data
    :param debug: bool, flag to show debug plots
    :return: float, closest object distance
    """
    ranges = np.array(scan.ranges)
    if (ranges <= 0).any() or np.inf in ranges:
        mask = np.logical_and(ranges >= 0, ranges < np.finfo(np.float64).max)
        # if False in mask:
        print('WARNING: invalid range values (negative of infinite value) in input LIDAR scan'
              'These values will be deleted, but please consider applying range filter first')
        ranges = ranges[mask]

    # histogram filter
    distance = np.min(ranges)   # simple method works better
    # hist = np.histogram(ranges, bins='auto')
    # distance = np.mean(ranges[np.logical_and(ranges >= hist[1][0], ranges <= hist[1][1])])

    if debug:
        hist = plt.hist(ranges, bins='auto')
        plt.plot([distance, distance], [0, np.max(hist[0])])
        plt.title('Calculated Distance: {}'.format(distance))
        plt.waitforbuttonpress(1)
        plt.draw()
        plt.close()

    return distance
