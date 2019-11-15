"""
Unit tests for LIDAR distance calculator
"""

import numpy as np

from sensor_msgs.msg import LaserScan
from lidar_distance_calculator import calculate_closest_object_distance


def _toy_scan_initializer():
    """
    Initialize toy LIDAR scan, leave ranges empty and fill in other values
    :returns: LaserScan msg, LIDAR scan data; int, number of scan readings
    """
    # generate toy laser scan
    num_readings = 90
    laser_frequency = 40

    scan = LaserScan()

    scan.header.stamp = 0
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = - np.pi / 4
    scan.angle_max = np.pi / 4
    scan.angle_increment = 1.0
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 10.0

    scan.ranges = []
    scan.intensities = [1] * num_readings
    return scan, num_readings


def test_distance_calculator_with_constant_distance():
    """
    Test distance calculator with constant distance
    """
    print('\n===== Testing with constant distance =====\n')
    scan, num_readings = _toy_scan_initializer()
    scan.ranges = np.ones(num_readings)

    print('sending scan range: ', scan.ranges)
    # test
    distance = calculate_closest_object_distance(scan, debug=True)
    print("Got distance: ", distance)
    assert distance == 1


def test_distance_calculator_with_binary_valued_distance():
    """
    Test distance calculator with binary valued distance
    """
    print('\n===== Testing with binary valued distance =====\n')
    scan, num_readings = _toy_scan_initializer()
    scan.ranges = np.random.randint(1, 3, size=num_readings)

    print('sending scan range: ', scan.ranges)
    # test
    distance = calculate_closest_object_distance(scan, debug=True)
    print("Got distance: ", distance)
    assert distance == 1


def test_distance_calculator_with_uniform_sampled_distance():
    """
    Test distance calculator with uniform sampled distance
    """
    print('\n===== Testing with uniform_sampled distance =====\n')
    scan, num_readings = _toy_scan_initializer()
    scan.ranges = np.random.uniform(0.5, 2, size=num_readings)

    print('sending scan range: ', scan.ranges)
    # test
    distance = calculate_closest_object_distance(scan, debug=True)
    print("Got distance: ", distance)
    assert 0.5 < distance <= 0.75


def test_distance_calculator_with_gaussian_sampled_distance():
    """
    Test distance calculator with Gaussian sampled distance
    """
    print('\n===== Testing with Gaussian sampled distance =====\n')
    scan, num_readings = _toy_scan_initializer()
    scan.ranges = np.random.normal(1, 0.1, size=num_readings)

    print('sending scan range: ', scan.ranges)
    # test
    distance = calculate_closest_object_distance(scan, debug=True)
    print("Got distance: ", distance)
    assert 0.65 < distance <= 0.9


def test_distance_calculator_with_inverse_gaussian_sampled_distance():
    """
    Test distance calculator with inverse Gaussian sampled distance
    """
    print('\n===== Testing with inverse Gaussian sampled distance =====\n')
    scan, num_readings = _toy_scan_initializer()
    scan.ranges = np.random.wald(1, 0.1, size=num_readings)

    print('sending scan range: ', scan.ranges)
    # test
    distance = calculate_closest_object_distance(scan, debug=True)
    print("Got distance: ", distance)
    assert 0 < distance <= 0.5


if __name__ == '__main__':
    print('Testing LIDAR distance calculator')
    test_distance_calculator_with_constant_distance()
    test_distance_calculator_with_binary_valued_distance()
    test_distance_calculator_with_uniform_sampled_distance()
    test_distance_calculator_with_gaussian_sampled_distance()
    test_distance_calculator_with_inverse_gaussian_sampled_distance()

    print('All tests passed')
