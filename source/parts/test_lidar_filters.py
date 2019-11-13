"""
Unit tests for LIDAR filters
"""

import time
import rospy
import numpy as np

from matplotlib import pyplot as plt
from sensor_msgs.msg import LaserScan

from lidar_filters import *


def test_angular_bounds_filter():
    """
    Test angular bounds filter
    """
    print('\n===== Testing angular bounds filter =====\n')
    # generate toy laser scan
    num_readings = 20
    laser_frequency = 40

    current_time = rospy.Time.now()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = 0
    scan.angle_max = num_readings
    scan.angle_increment = 1.0
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 100.0

    scan.ranges = []
    scan.intensities = []
    for i in range(0, num_readings):
        scan.ranges.append(scan.angle_min + i * scan.angle_increment)  # fake data
        scan.intensities.append(1)  # fake data
    print('\nBefore filter:\n', scan)

    # test filter
    lower_angle = 6
    upper_angle = 18
    print('*** Test: lower angle: {}, upper angle: {} ***'.format(lower_angle, upper_angle))
    angular_bounds_filter(scan, lower_angle, upper_angle)
    print('\nAfter angular bounds filter:\n', scan)

    assert scan.angle_min == lower_angle
    assert scan.angle_max == upper_angle
    assert scan.ranges[0] == lower_angle
    assert scan.ranges[-1] == upper_angle
    assert len(scan.ranges) == len(scan.intensities) == upper_angle - lower_angle + 1
    for i in range(len(scan.ranges) - 1):
        assert scan.ranges[i + 1] - scan.ranges[i] == scan.angle_increment


def test_range_filter():
    """
    Test range filter
    """
    print('\n===== Testing range filter =====\n')
    # generate toy laser scan
    num_readings = 20
    laser_frequency = 40

    current_time = rospy.Time.now()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = 3.14 / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 100.0

    scan.ranges = []
    scan.intensities = []
    for i in range(0, num_readings):
        scan.ranges.append(np.random.choice(a=[i, np.inf], p=[0.8, 0.2]))  # fake data
        scan.intensities.append(1)  # fake data
    print('\nBefore filter:\n', scan)

    # test filter
    lower_range = 0.5
    upper_range = 10
    print('*** Test: lower range: {}, upper range: {} ***'.format(lower_range, upper_range))
    range_filter(scan, lower_range, upper_range)
    print('\nAfter range filter:\n', scan)

    assert len(scan.ranges) == len(scan.intensities) == num_readings
    assert np.min(scan.ranges) >= lower_range
    assert np.max(scan.ranges) <= upper_range


def test_temporal_median_filter():
    """
    Test temporal median filter
    """
    print('\n===== Testing temporal median filter =====\n')
    # generate toy scans
    scan_list = []
    ranges_list = [[0, 1, 2, 1, 3],
                   [1, 5, 7, 1, 3],
                   [2, 3, 4, 1, 0],
                   [3, 3, 3, 1, 3],
                   [10, 2, 4, 0, 0]]

    for ranges in ranges_list:
        num_readings = len(ranges)
        laser_frequency = 40

        current_time = rospy.Time.now()

        scan = LaserScan()

        scan.header.stamp = current_time
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 3.14 / num_readings
        scan.time_increment = (1.0 / laser_frequency) / (num_readings)
        scan.range_min = 0.0
        scan.range_max = 100.0

        scan.ranges = ranges
        scan.intensities = [1] * len(ranges)
        scan_list.append(scan)

    # test temporal filter
    k = 0
    print('*** Test: k = {} ***'.format(k))
    temporal_median_filter = TemporalMedianFilter(k=k)
    ground_truth_list = [[0, 1, 2, 1, 3],
                         [1, 5, 7, 1, 3],
                         [2, 3, 4, 1, 0],
                         [3, 3, 3, 1, 3],
                         [10, 2, 4, 0, 0]]

    for i, scan in enumerate(scan_list):
        temporal_median_filter.filter(scan)
        print('result: ', scan.ranges)
        assert len(scan.ranges) == len(scan.intensities) == num_readings
        assert scan.ranges == tuple(ground_truth_list[i])

    k = 3
    print('*** Test: k = {} ***'.format(k))
    temporal_median_filter = TemporalMedianFilter(k=k)
    ground_truth_list = [[0, 1, 2, 1, 3],
                         [0.5, 3, 4.5, 1, 3],
                         [1, 3, 4, 1, 3],
                         [1.5, 3, 3.5, 1, 3],
                         [2.5, 3, 4, 1, 1.5]]

    for i, scan in enumerate(scan_list):
        temporal_median_filter.filter(scan)
        print('result: ', scan.ranges)
        assert len(scan.ranges) == len(scan.intensities) == num_readings
        assert scan.ranges == tuple(ground_truth_list[i])


def test_filter_stress_test():
    """
    Stress test of filters and speed check
    """
    print('\n====== Stress test ======\n')
    # generate toy laser scan
    num_readings = 100
    laser_frequency = 40

    current_time = rospy.Time.now()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = 3.14 / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 100.0

    scan.ranges = []
    scan.intensities = []
    for i in range(0, num_readings):
        scan.ranges.append(i)  # fake data
        scan.intensities.append(1)  # fake data

    # angular bounds filter speed test
    time_start = time.time()
    for i in range(10000):
        angular_bounds_filter(scan, -np.pi / 4, np.pi / 4)
    time_end = time.time()
    print('*** Angular filter run time: {} sec over {} runs, FPS = {} ***'.format(time_end - time_start, i + 1,
                                                                                  ((i + 1) / (time_end - time_start))))

    # range filter speed test
    time_start = time.time()
    for i in range(10000):
        range_filter(scan, 0.1, 10)
    time_end = time.time()
    print('*** Range filter run time: {} sec over {} runs, FPS = {} ***'.format(time_end - time_start, i + 1,
                                                                                ((i + 1) / (time_end - time_start))))

    # temporal median filter speed test
    temporal_median_filter = TemporalMedianFilter(k=5)
    time_start = time.time()
    for i in range(10000):
        temporal_median_filter.filter(scan)
    time_end = time.time()
    print('*** Temporal median filter run time: {} sec over {} runs, FPS = {} ***'.format(time_end - time_start, i + 1,
                                                                                          ((i + 1) / (
                                                                                                      time_end - time_start))))


if __name__ == '__main__':
    rospy.init_node('test_lidar_filters', anonymous=True)  # init ROS node

    test_angular_bounds_filter()
    test_range_filter()
    test_temporal_median_filter()
    test_filter_stress_test()

    print('All tests passed')
