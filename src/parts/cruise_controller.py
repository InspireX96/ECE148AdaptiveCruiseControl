"""Adaptive Cruise Controller"""

import logging
import numpy as np


class CruiseController(object):
    """
    Adaptive cruise controller as a donkey part.

    The cruise controller takes distance as input and calculate throttle to keep distance constant
    """

    def __init__(self, default_distance=0.5):
        """
        Constructor of CruiseController
        :param default_distance: float, default distance to keep, defaults to 0.5 (m)
        """
        self.default_distance = default_distance

        self.throttle = 0
        self.last_throttle = 0
        self.last_distance = 0
        self.time_step = 0.05
        self.violence_scale = 1.5   # TODO: tune parameter
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
        # calculate distance error
        # TODO: PID?
        # TODO: set timer
        self.error = distance - self.default_distance + self.violence_scale * (distance -
                                                                               self.last_distance) / self.time_step

        # change throttle if error is beyond threshold
        if self.error > self.error_high_threshold or self.error < self.error_low_threshold:
            self.throttle += self.error * self.throttle_change

        # clip throttle
        # TODO: tanh to smooth out?
        self.throttle = np.clip(self.throttle, self.min_throttle, self.max_throttle)

        # update memory
        self.last_distance = distance
        self.last_throttle = self.throttle

        print('Throttle before clip: ', self.throttle)

        return self.throttle

    def shutdown(self):
        pass
