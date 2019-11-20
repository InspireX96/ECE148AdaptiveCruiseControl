"""
Unit tests for Cruise Controller
"""

import time
import numpy as np
import matplotlib.pyplot as plt

from cruise_controller import CruiseController


def _cruise_controller_test_helper(cruise_controller, distance_list):
    """
    Helper function for cruise controller unit test
    Test cruise controller using given distance and plot

    :param cruise_controller: object, CruiseController object
    :param distance_list: array-like, list of distances
    :returns: list of throttle and error
    """
    throttle_list = []
    error_list = []
    for distance in distance_list:
        throttle = cruise_controller.run(distance)
        throttle_list.append(throttle)
        error = cruise_controller.error
        error_list.append(error)

    fig, (ax1, ax3) = plt.subplots(1, 2)
    color = 'tab:red'
    ax1.set_xlabel('time ({}s)'.format(cruise_controller.time_step))
    ax1.set_ylabel('distance (m)', color=color)
    ax1.plot(distance_list, color=color)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    color = 'tab:blue'
    ax2.set_ylabel('throttle', color=color)
    ax2.plot(throttle_list, color=color)
    ax2.tick_params(axis='y', labelcolor=color)

    ax3.set_xlabel('time ({}s)'.format(cruise_controller.time_step))
    ax3.set_ylabel('error')
    ax3.plot(error_list)

    plt.waitforbuttonpress(10)
    plt.draw()
    plt.close()
    return throttle_list, error_list


def test_cruise_controller_with_constant_default_distance():
    """
    Test cruise controller with constant default distance
    """
    print('\n===== Testing with constant default distance =====\n')
    default_distance = 0.5

    cruise_controller = CruiseController(default_distance=default_distance)

    distance_list = [default_distance] * int(1 / cruise_controller.time_step)
    throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)
    print('Throttle list: ', throttle_list)
    print('Error list: ', error_list)

    # TODO: assert


def test_cruise_controller_with_constant_non_default_distance():
    """
    Test cruise controller with constant non default distance
    """
    print('\n===== Testing with constant non default distance =====\n')
    default_distance = 0.5

    cruise_controller = CruiseController(default_distance=default_distance)
    distance_list = [2 * default_distance] * int(1 / cruise_controller.time_step)
    throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)
    print('Throttle list: ', throttle_list)
    print('Error list: ', error_list)

    cruise_controller = CruiseController(default_distance=default_distance)

    distance_list = [0.5 * default_distance] * int(1 / cruise_controller.time_step)
    throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)
    print('Throttle list: ', throttle_list)
    print('Error list: ', error_list)
    # TODO: assert


def test_cruise_controller_with_increasing_to_default_distance():
    """
    Test cruise controller with increasing to default distance
    """
    print('\n===== Testing with increasing to default distance =====\n')
    default_distance = 0.5

    cruise_controller = CruiseController(default_distance=default_distance)

    x = np.linspace(-10, 10, int(1 / cruise_controller.time_step))
    distance_list = 1/(1 + np.exp(-x)) * default_distance

    throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)
    print('Throttle list: ', throttle_list)
    print('Error list: ', error_list)


def test_cruise_controller_with_decreasing_to_default_distance():
    """
    Test cruise controller with decreasing to default distance
    """
    print('\n===== Testing with decreasing to default distance =====\n')
    default_distance = 0.5

    cruise_controller = CruiseController(default_distance=default_distance)

    x = np.linspace(-10, 10, int(1 / cruise_controller.time_step))
    distance_list = 1/(1 + np.exp(-x)) * default_distance
    distance_list = distance_list[::-1]

    throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)
    print('Throttle list: ', throttle_list)
    print('Error list: ', error_list)


def test_cruise_controller_with_constant_sin_wave_distance():
    """
    Test cruise controller with constant sin wave distance
    """
    print('\n===== Testing with constant sin wave distance =====\n')
    default_distance = 0.5

    cruise_controller = CruiseController(default_distance=default_distance)
    distance_list = [default_distance * np.sin(i / 20) + 0.5 for i in range(int(10 / cruise_controller.time_step))]

    throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)
    print('Throttle list: ', throttle_list)
    print('Error list: ', error_list)

    # TODO: assert


if __name__ == '__main__':
    print('Testing cruise controller')
    test_cruise_controller_with_constant_default_distance()
    test_cruise_controller_with_constant_non_default_distance()
    test_cruise_controller_with_increasing_to_default_distance()
    test_cruise_controller_with_decreasing_to_default_distance()
    test_cruise_controller_with_constant_sin_wave_distance()
    print('All tests passed')
