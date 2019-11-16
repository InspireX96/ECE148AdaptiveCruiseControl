"""
LIDAR Processor, receive LIDAR scan msg, apply filters, calculate distance
"""

import time
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

from lidar_filters import angular_bounds_filter, range_filter, TemporalMedianFilter
from lidar_distance_calculator import calculate_closest_object_distance


class LidarProcessor(object):
    """
    LIDAR Processor as a donkey part.

    This processor will subscribe to ROS topic '/scan',
        apply filters to LIDAR scans to reduce noise,
        return closest obstacle distance in front of the donkey car.
    """

    def __init__(self):
        # init ROS node
        rospy.init_node('lidar_processor', anonymous=True)
        self.calculated_distance = 0
        self.temporal_median_filter = TemporalMedianFilter(k=3)  # initialize temporal median filter

    @staticmethod
    def _get_lidar_scan():
        """
        Get LIDAR scan from ROS message
        :return: LaserScan msg, LIDAR scan data
        """
        try:
            scan = rospy.wait_for_message('scan', LaserScan, timeout=1)
            return scan

        except Exception as err:
            print('Timeout receiving LIDAR data: {}\n'
                  'Please check LIDAR data publisher is publishing LIDAR scans'.format(err))
            return None

    def _apply_lidar_filters(self, scan):
        """
        Apply series of LIDAR filters to reduce noise
        :param scan: LaserScan msg, LIDAR scan data
        :return: LaserScan msg, filtered LIDAR scan data
        """
        # TODO: check filter parameters
        scan = angular_bounds_filter(scan, np.pi + -np.pi / 4, np.pi + np.pi / 4)
        scan = range_filter(scan, 0, 5)
        scan = self.temporal_median_filter(scan)
        return scan

    def run(self):
        """
        Run as a non-threaded script.
        Receive LIDAR scan, apply filters and calculate distance

        :return: float, calculated distance from LIDAR scan
        """
        scan = self._get_lidar_scan()
        if scan is None:
            self.calculated_distance = 0
            print('Cannot receive LIDAR data, set distance to {}'.format(self.calculated_distance))
        else:
            self._apply_lidar_filters(scan)  # apply filters
            self.calculated_distance = calculate_closest_object_distance(scan)  # calculate distance
        return self.calculated_distance

    def shutdown(self):
        pass


if __name__ == '__main__':
    # test LIDAR processor
    lidar_processor = LidarProcessor()

    for _ in range(100):
        time_start = time.time()
        distance = lidar_processor.run()
        time_elapsed = time.time() - time_start
        print('Distance: {}, elapsed time: {}, fps: {}'.format(distance, time_elapsed, 1 / time_elapsed))
