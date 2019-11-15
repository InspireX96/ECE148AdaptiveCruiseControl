"""
LIDAR Processor, receive LIDAR scan msg, apply filters, calculate distance
"""

class LidarProcessor(object):
    """
    LIDAR Processor as a donkey part.

    This Processor will subscribe to ROS topic '/scan',
        apply filters to LIDAR scans to reduce noise,
        return closest obstacle distance in front of the donkey car.
    """
    def __init__(self):
        self.calculated_distance = 0

    def update(self):
        pass

    def run_threaded(self):
        """
        Run as a threaded script.
        :return:
        """
        pass

    def run(self):
        """
        Run as a non-threaded script.
        :return:
        """
        pass

    def shutdown(self):
        pass