"""
Calculate closed object distance from LIDAR scans
"""
import numpy as np

def calculate_closest_object_distance(scan):
    """
    This function calculates object distance from given LIDAR scan.

    :param scan: LaserScan msg, LIDAR scan data
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

    return distance
