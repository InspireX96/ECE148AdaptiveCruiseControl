#!/usr/bin/env python3
"""
Modified scripts to drive a donkey 2 car with adaptive cruise control
NOTE: train mode is deleted in this script, please use original manage.py to train model

Usage:
    manage.py (drive) [--model=<model>] [--js] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer|latent)] [--camera=(single|stereo)] [--meta=<key:value> ...]

Options:
    -h --help          Show this screen.
    --js               Use physical joystick.
    -f --file=<file>   A text file containing paths to tub files, one per line. Option may be used more than once.
    --meta=<key:value> Key/Value strings describing describing a piece of meta data about this drive. Option may be used more than once.
"""

import logging
from docopt import docopt
import donkeycar as dk

# from donkeycar.parts.controller import LocalWebController, JoystickController, get_js_controller, JoyStickSub
# from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

from donkeycar.parts.lidar_processor import LidarProcessor


def drive(cfg, model_path=None, use_joystick=False, model_type=None, camera_type='single', meta=[]):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    # initialize car
    V = dk.vehicle.Vehicle()

    # TODO: setup camera

    # # setup joystick/web controller
    # if use_joystick or cfg.USE_JOYSTICK_AS_DEFAULT:
    #     # modify max_throttle closer to 1.0 to have more power
    #     # modify steering_scale lower than 1.0 to have less responsive steering
    #     logging.warning('Using joystick controller')
    #
    #     ctr = get_js_controller(cfg)
    #
    #     if cfg.USE_NETWORKED_JS:
    #         logging.warning('Using networked joystick controller')
    #         netwkJs = JoyStickSub(cfg.NETWORK_JS_SERVER_IP)
    #         V.add(netwkJs, threaded=True)
    #         ctr.js = netwkJs
    #
    # else:
    #     # This web controller will create a web server that is capable
    #     # of managing steering, throttle, and modes, and more.
    #     logging.warning('Using web controller')
    #     ctr = LocalWebController()
    #
    # V.add(ctr,
    #       inputs=[],
    #       outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
    #       threaded=True)  # NOTE: user mode is given by controller, which can change DriveMode
    #
    # # this throttle filter will allow one tap back for esc reverse
    # th_filter = ThrottleFilter()
    # V.add(th_filter, inputs=['user/throttle'], outputs=['user/throttle'])

    # add LIDAR stuff
    lidar_processor = LidarProcessor(non_block=False, debug=True)  # use non-block mode for more fps
    V.add(lidar_processor, outputs=['lidar/distance'], threaded=False)

    # TODO: autopilot
    # TODO: load model

    # class DriveMode:
    #     """
    #     Change different drive modes: manual, adaptive CC, autopilot, etc...
    #     """
    #
    #     # TODO: change drive mode and controller.py
    #     def run(self, mode,
    #             user_angle, user_throttle,
    #             pilot_angle, pilot_throttle):
    #         # TODO: add adaptive CC throttle here
    #         if mode == 'user':
    #             return user_angle, user_throttle
    #
    #         elif mode == 'local_angle':
    #             logging.error('Autopilot not implemented!')
    #             return pilot_angle, user_throttle
    #
    #         else:
    #             logging.error('Autopilot not implemented!')
    #             return pilot_angle, pilot_throttle * cfg.AI_THROTTLE_MULT
    #
    # V.add(DriveMode(),
    #       inputs=['user/mode', 'user/angle', 'user/throttle',
    #               'pilot/angle', 'pilot/throttle'],
    #       outputs=['angle', 'throttle'])
    #
    # # setup drive train
    # steering_controller = PCA9685(
    #     cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    # steering = PWMSteering(controller=steering_controller,
    #                        left_pulse=cfg.STEERING_LEFT_PWM,
    #                        right_pulse=cfg.STEERING_RIGHT_PWM)
    #
    # throttle_controller = PCA9685(
    #     cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    # throttle = PWMThrottle(controller=throttle_controller,
    #                        max_pulse=cfg.THROTTLE_FORWARD_PWM,
    #                        zero_pulse=cfg.THROTTLE_STOPPED_PWM,
    #                        min_pulse=cfg.THROTTLE_REVERSE_PWM)
    #
    # V.add(steering, inputs=['angle'])
    # V.add(throttle, inputs=['throttle'])

    # run the vehicle with drive loop frequency and max loops defined in config.py
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ,
            max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)  # add argparse
    cfg = dk.load_config()  # load myconfig.py or config.py

    # drive the vehicle
    if args['drive']:
        model_type = args['--type']
        camera_type = args['--camera']
        drive(cfg, model_path=args['--model'], use_joystick=args['--js'], model_type=model_type,
              camera_type=camera_type,
              meta=args['--meta'])
