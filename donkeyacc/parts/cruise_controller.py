"""
Adaptive Cruise Controller
"""

import time
import logging
import numpy as np


class CruiseController(object):
    """
    Adaptive cruise controller as a donkey part.

    The cruise controller takes distance as input and calculate throttle to keep distance constant
    """

    def __init__(self, kp=1.0, kd=1.5, default_distance=0.5, throttle_scale=1.0, max_throttle=1.0, time_step=0.05, use_timer=False, debug=False):
        """
        Constructor of CruiseController
        :param kp: float, proportional gain, defaults to 1
        :param kd: float, differential gain, defaults to 1.5
        :param default_distance: float, default distance to keep, defaults to 0.5 (m)
        :param throttle_scale: float, scale factor of output throttle, normally defined in myconfig.py
        :param max_throttle: float (-1~1), initial maximum throttle, defaults to 1.0,
                             can be changed by user input later on
        :param time_step: float, time step (sec), defaults to 0.5
        :param use_timer: bool, flag to use a timer to calculate time step instead of setting a constant value
        :param debug: bool, flag to turn on debug mode that prints out calculated distance
        """
        # TODO: scale throttle based on config
        # controller parameters
        self.kp = kp
        self.kd = kd  # TODO: kd may be too high
        # controller settings
        self.default_distance = default_distance
        self.throttle_scale = throttle_scale
        self.time_step = time_step
        self.use_timer = use_timer
        self.debug = debug

        self.throttle = 0
        self.last_throttle = 0
        self.last_distance = 0
        self.error = 0
        self.throttle_change = 0.25  # TODO: maybe tune down this?
        self.error_high_threshold = 0.05
        self.error_low_threshold = -0.05
        self.max_throttle = max_throttle
        self.min_throttle = -1

        # setup timer stuff
        self.timer_change_max_throttle = time.time()
        self.timer_time_step = time.time()
        self.sleep_time_change_max_throttle = 0.5  # user throttle input will be ignored within sleep time

    def _change_max_throttle(self, user_throttle):
        """
        Change max_throttle based on user input
        NOTE: this function does not return but changes self.max_throttle
        :param user_throttle: user input throttle to adjust controller, defaults to None
        """
        if user_throttle is not None and (
                time.time() - self.timer_change_max_throttle) > self.sleep_time_change_max_throttle:
            # change max throttle limit according to user throttle input
            if user_throttle > 0.8 * self.throttle_scale and self.max_throttle + self.throttle_change <= 1:
                self.max_throttle += self.throttle_change
                print('Cruise control max throttle increased {} to {}'.format(self.throttle_change, self.max_throttle))
            elif user_throttle < -0.8 * self.throttle_scale and \
                    self.max_throttle - self.throttle_change >= self.min_throttle:
                self.max_throttle -= self.throttle_change
                print('Cruise control max throttle decreased {} to {}'.format(self.throttle_change, self.max_throttle))
            else:
                print('Cannot change cruise control max throttle')
            self.timer_change_max_throttle = time.time()

    def run(self, distance, user_throttle=None):
        """
        Run as a non-threaded script.
        Input distance, output throttle
        NOTE: increase max throttle setting by giving positive user_throttle input and vice versa

        :param distance: float, distance
        :param user_throttle: user input throttle to adjust controller, defaults to None
        :return: float, throttle
        """
        # calculate distance error using PD controller
        # TODO: set timer
        # self._change_max_throttle(user_throttle)  # change max throttle according to user throttle input
        if self.use_timer:
            time_interval = time.time() - self.timer_time_step
            self.timer_time_step = time.time()
        else:
            time_interval = self.time_step

        self.error = self.kp * (distance - self.default_distance) + self.kd * (distance -
                                                                               self.last_distance) / time_interval

        # change throttle if error is beyond threshold
        if self.error > self.error_high_threshold or self.error < self.error_low_threshold:
            self.throttle += self.error * self.throttle_change

        # clip throttle
        self.throttle = np.clip(self.throttle, self.min_throttle, self.max_throttle)

        # update memory
        self.last_distance = distance
        self.last_throttle = self.throttle

        logging.info('output throttle: {} * {} (scale)'.format(self.throttle, self.throttle_scale))
        if self.debug:
            print('output throttle: {} * {} (scale)'.format(self.throttle, self.throttle_scale))

        return self.throttle * self.throttle_scale

    def shutdown(self):
        pass
