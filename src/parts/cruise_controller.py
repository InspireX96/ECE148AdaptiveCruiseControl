"""Adaptive Cruise Controller"""

import logging
import numpy as np


class CruiseController(object):
    """
    Adaptive cruise controller as a donkey part.

    The cruise controller takes distance as input and calculate throttle to keep distance constant
    """

    def __init__(self, kp=1, kd=1.5, default_distance=0.5, debug=False):
        """
        Constructor of CruiseController
        :param kp: float, proportional gain, defaults to 1
        :param kd: float, differential gain, defaults to 1.5
        :param default_distance: float, default distance to keep, defaults to 0.5 (m)
        :param debug: bool, flag to turn on debug mode that prints out calculated distance
        """
        # controller parameters
        self.kp = kp
        self.kd = kd
        # controller settings
        self.default_distance = default_distance
        self.debug = debug

        self.throttle = 0
        self.last_throttle = 0
        self.last_distance = 0
        self.time_step = 0.05
        self.error = 0
        self.throttle_change = 0.25
        self.error_high_threshold = 0.1
        self.error_low_threshold = -0.1
        self.max_throttle = 1
        self.min_throttle = -1

    def run(self, distance):
        """
        Run as a non-threaded script.
        Input distance, output throttle

        :param distance: float, distance
        :return: float, throttle
        """
        # calculate distance error using PD controller
        # TODO: set timer
        self.error = self.kp * (distance - self.default_distance) + self.kd * (distance -
                                                                               self.last_distance) / self.time_step

        # change throttle if error is beyond threshold
        if self.error > self.error_high_threshold or self.error < self.error_low_threshold:
            self.throttle += self.error * self.throttle_change

        # clip throttle
        self.throttle = np.clip(self.throttle, self.min_throttle, self.max_throttle)

        # update memory
        self.last_distance = distance
        self.last_throttle = self.throttle

        logging.info('Output throttle: {}', self.throttle)
        if self.debug:
            print('Output throttle: {}', self.throttle)

        return self.throttle

    def shutdown(self):
        pass
