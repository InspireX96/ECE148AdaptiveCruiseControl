"""
LIDAR Filters
"""
from collections import deque
import numpy as np


def angular_bounds_filter(scan, lower_angle, upper_angle):
    """
    Angular bounds filter
    Bound the scans between lower and upper angle

    NOTE: It will modify the scan msg reference in place

    :param scan: LaserScan msg, LIDAR scan data
    :param lower_angle: float, lower angle bound of start angle of the scan (rad)
    :param upper_angle: float, upper angle bound of end angle of the scan (rad)
    :return: LaserScan msg, LIDAR scan data.
             NOTE: input data is already modified, this is just for usage convenience
    """
    assert lower_angle >= scan.angle_min
    assert upper_angle <= scan.angle_max

    # calculate lower and upper index
    scan_lower_index = int((lower_angle - scan.angle_min) // scan.angle_increment)
    scan_upper_index = int(len(scan.ranges) + 1 - (scan.angle_max - upper_angle) // scan.angle_increment)

    # update scan msg
    scan.angle_min = lower_angle
    scan.angle_max = upper_angle
    scan.ranges = scan.ranges[scan_lower_index:scan_upper_index]
    scan.intensities = scan.intensities[scan_lower_index:scan_upper_index]
    return scan


def range_filter(scan, lower_range, upper_range):
    """
    Range filter
    Clip the range of scans between lower and upper range

    NOTE: It will modify the scan msg reference in place

    :param scan: LaserScan msg, LIDAR scan data
    :param lower_range: float, lower range threshold (m)
    :param upper_range: float, upper range threshold (m)
    :return: LaserScan msg, LIDAR scan data.
             NOTE: input data is already modified, this is just for usage convenience
    """
    assert upper_range >= lower_range >= 0

    ranges = np.array(scan.ranges)

    # clip ranges
    ranges = np.clip(ranges, lower_range, upper_range)

    # update scan msg
    scan.ranges = tuple(ranges)
    return scan


class TemporalMedianFilter(object):
    """
    Temporal median filter
    Return the median of the current and previous k scans
    """

    def __init__(self, k):
        """
        Create temporal buffer for the filter
        Please init this class before the filter scan

        :param k: int, previous number of scans to be countered
        """
        assert isinstance(k, int) and k >= 0
        self.k = k
        self._scan_buffer = deque()  # buffer to scan temporal scans

    def filter(self, scan):
        """
        Filter one scan using temporal median

        NOTE: It will modify the scan msg reference in place

        :param scan: LaserScan msg, LIDAR scan data
        :return: LaserScan msg, LIDAR scan data.
                 NOTE: input data is already modified, this is just for usage convenience
        """
        ranges = np.array(scan.ranges)

        # store temporal scans
        self._scan_buffer.append(ranges)

        # maintain buffer size
        if len(self._scan_buffer) > self.k + 1:
            self._scan_buffer.popleft()

        previous_ranges = np.asarray(self._scan_buffer)
        result_ranges = np.median(previous_ranges, axis=0)

        # update scan msg
        scan.ranges = tuple(result_ranges)
        return scan
