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

    def __init__(self, non_block=False):
        """
        Constructor of LidarProcessor
        :param non_block: bool, flag to turn on non-block mode.
                          If non_block=False, the processor will block the process while waiting for a new LIDAR scan.
                              Once a new scan is received, it will calculate distance and return back.
                          If non_block=True, the processor will send back previous calculated distance while
                              waiting for a new LIDAR scan, hence it will not block the process.
        """
        self.non_block = non_block
        # init ROS node
        rospy.init_node('lidar_processor', anonymous=True)
        self.calculated_distance = 0
        self.temporal_median_filter = TemporalMedianFilter(k=3)  # initialize temporal median filter

    def _get_lidar_scan(self):
        """
        Get LIDAR scan from ROS message
        :return: LaserScan msg, LIDAR scan data
        """
        try:
            if not self.non_block:
                scan = rospy.wait_for_message('scan', LaserScan, timeout=1)
            else:
                scan = rospy.wait_for_message('scan', LaserScan, timeout=0.05)  # TODO: tune this
            return scan

        except Exception as err:
            if not self.non_block:
                print('Timeout receiving LIDAR data: {}'.format(err))
                self.calculated_distance = 0  # if timeout in block mode, we set distance to 0 for safety
                print('Cannot receive LIDAR data, set distance to {}'.format(self.calculated_distance))
            # in non_block mode, we just use the previous calculated distance
            return None

    def _apply_lidar_filters(self, scan):
        """
        Apply series of LIDAR filters to reduce noise
        :param scan: LaserScan msg, LIDAR scan data
        :return: LaserScan msg, filtered LIDAR scan data
        """
        # TODO: check filter parameters
        scan = angular_bounds_filter(scan, np.pi - np.pi / 6, np.pi + np.pi / 6)
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
        if scan is not None:
            self._apply_lidar_filters(scan)  # apply filters
            self.calculated_distance = calculate_closest_object_distance(scan)  # calculate distance
        # TODO: maybe try to smooth out distance in non block mode
        return self.calculated_distance

    def shutdown(self):
        pass


if __name__ == '__main__':
    # test LIDAR processor
    # test block mode
    lidar_processor = LidarProcessor()
    for _ in range(100):
        time_start = time.time()
        distance = lidar_processor.run()
        time_elapsed = time.time() - time_start
        print('Distance: {}, elapsed time: {}, fps: {}'.format(distance, time_elapsed, 1 / time_elapsed))

    # test non block mode
    lidar_processor = LidarProcessor(non_block=True)
    for _ in range(100):
        time_start = time.time()
        distance = lidar_processor.run()
        time_elapsed = time.time() - time_start
        print('Distance: {}, elapsed time: {}, fps: {}'.format(distance, time_elapsed, 1 / time_elapsed))
