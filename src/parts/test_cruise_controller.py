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

    plt.waitforbuttonpress(1)
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

    assert np.min(throttle_list) >= cruise_controller.min_throttle
    assert np.max(throttle_list) <= cruise_controller.max_throttle
    assert abs(np.mean(error_list[1:])) <= 0.5
    assert abs(error_list[-1]) <= 0.5
    assert np.var(error_list[1:]) <= 1


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

    assert np.min(throttle_list) >= cruise_controller.min_throttle
    assert np.max(throttle_list) <= cruise_controller.max_throttle
    assert abs(np.mean(error_list[1:])) <= 0.5
    assert abs(error_list[-1]) <= 0.5
    assert np.var(error_list[1:]) <= 1

    cruise_controller = CruiseController(default_distance=default_distance)

    distance_list = [0.5 * default_distance] * int(1 / cruise_controller.time_step)
    throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)
    print('Throttle list: ', throttle_list)
    print('Error list: ', error_list)

    assert np.min(throttle_list) >= cruise_controller.min_throttle
    assert np.max(throttle_list) <= cruise_controller.max_throttle
    assert abs(np.mean(error_list[1:])) <= 0.5
    assert abs(error_list[-1]) <= 0.5
    assert np.var(error_list[1:]) <= 1


def test_cruise_controller_with_increasing_to_default_distance():
    """
    Test cruise controller with increasing to default distance
    """
    print('\n===== Testing with increasing to default distance =====\n')
    default_distance = 0.5

    cruise_controller = CruiseController(default_distance=default_distance)

    x = np.linspace(-10, 10, int(1 / cruise_controller.time_step))
    distance_list = 1 / (1 + np.exp(-x)) * default_distance

    throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)
    print('Throttle list: ', throttle_list)
    print('Error list: ', error_list)

    assert np.min(throttle_list) >= cruise_controller.min_throttle
    assert np.max(throttle_list) <= cruise_controller.max_throttle
    assert abs(np.mean(error_list[1:])) <= 2
    assert abs(error_list[-1]) <= 1
    assert np.var(error_list[1:]) <= 2


def test_cruise_controller_with_decreasing_to_default_distance():
    """
    Test cruise controller with decreasing to default distance
    """
    print('\n===== Testing with decreasing to default distance =====\n')
    default_distance = 0.5

    cruise_controller = CruiseController(default_distance=default_distance)

    x = np.linspace(-10, 10, int(1 / cruise_controller.time_step))
    distance_list = 1 / (1 + np.exp(-x)) * default_distance
    distance_list = distance_list[::-1]

    throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)
    print('Throttle list: ', throttle_list)
    print('Error list: ', error_list)

    assert np.min(throttle_list) >= cruise_controller.min_throttle
    assert np.max(throttle_list) <= cruise_controller.max_throttle
    assert abs(np.mean(error_list[1:])) <= 2
    assert abs(error_list[-1]) <= 1
    assert np.var(error_list[1:]) <= 2


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

    assert np.min(throttle_list) >= cruise_controller.min_throttle
    assert np.max(throttle_list) <= cruise_controller.max_throttle
    assert abs(np.mean(error_list[1:])) <= 2
    assert abs(error_list[-1]) <= 1
    assert np.var(error_list[1:]) <= 2


def test_cruise_controller_set_throttle_scale():
    """
    Test cruise controller set throttle scale with random distance
    """
    print('\n===== Testing setting throttle scale with random distance =====\n')
    default_distance = 0.5
    for throttle_scale in [0.2, 0.5, 0.75]:
        cruise_controller = CruiseController(default_distance=default_distance, throttle_scale=throttle_scale,
                                             debug=True)
        distance_list = np.random.rand(int(1 / cruise_controller.time_step))

        throttle_list, error_list = _cruise_controller_test_helper(cruise_controller, distance_list)

        assert np.max(np.abs(throttle_list)) <= throttle_scale


def test_cruise_controller_stress_test():
    """
    Stress test of cruise controller and speed check
    """
    print('\n====== Stress test ======\n')
    cruise_controller = CruiseController()
    distance_mat = np.random.random((100, int(1 / cruise_controller.time_step)))
    time_start = time.time()
    for i, distance_list in enumerate(distance_mat):
        for distance in distance_list:
            cruise_controller.run(distance)
    time_end = time.time()
    print('*** Cruise controller run time: {} sec over {} runs with distance array len = {}, FPS = {} ***' \
          .format(time_end - time_start, i + 1, distance_mat.shape[1], ((i + 1) / (time_end - time_start))))


if __name__ == '__main__':
    print('Testing cruise controller')
    test_cruise_controller_with_constant_default_distance()
    test_cruise_controller_with_constant_non_default_distance()
    test_cruise_controller_with_increasing_to_default_distance()
    test_cruise_controller_with_decreasing_to_default_distance()
    test_cruise_controller_with_constant_sin_wave_distance()
    test_cruise_controller_set_throttle_scale()
    test_cruise_controller_stress_test()
    print('All tests passed')

