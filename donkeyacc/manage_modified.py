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

from donkeycar.parts.controller import LocalWebController, JoystickController, get_js_controller, JoyStickSub
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.behavior import BehaviorPart
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

from donkeycar.parts.lidar_processor import LidarProcessor
from donkeycar.parts.cruise_controller import CruiseController
from donkeycar.utils import *


def drive(cfg, model_path=None, use_joystick=False, model_type=None, camera_type='single', meta=[]):
    """
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    """
    if model_type is None:
        if cfg.TRAIN_LOCALIZER:
            model_type = "localizer"
        elif cfg.TRAIN_BEHAVIORS:
            model_type = "behavior"
        else:
            model_type = cfg.DEFAULT_MODEL_TYPE

    # initialize car
    V = dk.vehicle.Vehicle()

    # setup camera
    if camera_type == "stereo":

        if cfg.CAMERA_TYPE == "WEBCAM":
            from donkeycar.parts.camera import Webcam

            camA = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam=0)
            camB = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam=1)

        elif cfg.CAMERA_TYPE == "CVCAM":
            from donkeycar.parts.cv import CvCam

            camA = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam=0)
            camB = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam=1)
        else:
            raise (Exception("Unsupported camera type: %s" % cfg.CAMERA_TYPE))

        V.add(camA, outputs=['cam/image_array_a'], threaded=True)
        V.add(camB, outputs=['cam/image_array_b'], threaded=True)

        from donkeycar.parts.image import StereoPair

        V.add(StereoPair(), inputs=['cam/image_array_a', 'cam/image_array_b'],
              outputs=['cam/image_array'])

    else:
        print("cfg.CAMERA_TYPE", cfg.CAMERA_TYPE)
        if cfg.DONKEY_GYM:
            from donkeycar.parts.dgym import DonkeyGymEnv

        inputs = []
        threaded = True
        print("cfg.CAMERA_TYPE", cfg.CAMERA_TYPE)
        if cfg.DONKEY_GYM:
            from donkeycar.parts.dgym import DonkeyGymEnv
            cam = DonkeyGymEnv(cfg.DONKEY_SIM_PATH, env_name=cfg.DONKEY_GYM_ENV_NAME)
            threaded = True
            inputs = ['angle', 'throttle']
        elif cfg.CAMERA_TYPE == "PICAM":
            from donkeycar.parts.camera import PiCamera
            cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        elif cfg.CAMERA_TYPE == "WEBCAM":
            from donkeycar.parts.camera import Webcam
            cam = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        elif cfg.CAMERA_TYPE == "CVCAM":
            from donkeycar.parts.cv import CvCam
            cam = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        elif cfg.CAMERA_TYPE == "CSIC":
            from donkeycar.parts.camera import CSICamera
            cam = CSICamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH,
                            framerate=cfg.CAMERA_FRAMERATE, gstreamer_flip=cfg.CSIC_CAM_GSTREAMER_FLIP_PARM)
        elif cfg.CAMERA_TYPE == "V4L":
            from donkeycar.parts.camera import V4LCamera
            cam = V4LCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH,
                            framerate=cfg.CAMERA_FRAMERATE)
        elif cfg.CAMERA_TYPE == "MOCK":
            from donkeycar.parts.camera import MockCamera
            cam = MockCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        else:
            raise (Exception("Unkown camera type: %s" % cfg.CAMERA_TYPE))

        V.add(cam, inputs=inputs, outputs=['cam/image_array'], threaded=threaded)

    # setup joystick/web controller
    if use_joystick or cfg.USE_JOYSTICK_AS_DEFAULT:
        # modify max_throttle closer to 1.0 to have more power
        # modify steering_scale lower than 1.0 to have less responsive steering
        logging.warning('Using joystick controller')

        ctr = get_js_controller(cfg)

        if cfg.USE_NETWORKED_JS:
            logging.warning('Using networked joystick controller')
            netwkJs = JoyStickSub(cfg.NETWORK_JS_SERVER_IP)
            V.add(netwkJs, threaded=True)
            ctr.js = netwkJs

    else:
        # This web controller will create a web server that is capable
        # of managing steering, throttle, and modes, and more.
        logging.warning('Using web controller')
        ctr = LocalWebController()

    V.add(ctr,
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

    # this throttle filter will allow one tap back for esc reverse
    th_filter = ThrottleFilter()
    V.add(th_filter, inputs=['user/throttle'], outputs=['user/throttle'])

    # See if we should even run the pilot module.
    # This is only needed because the part run_condition only accepts boolean
    class PilotCondition:
        def run(self, mode):
            if mode == 'user':
                return False
            else:
                return True

    V.add(PilotCondition(), inputs=['user/mode'], outputs=['run_pilot'])

    V.add(ctr,
          inputs=[],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)  # NOTE: user mode is given by controller, which can change DriveMode

    # this throttle filter will allow one tap back for esc reverse
    th_filter = ThrottleFilter()
    V.add(th_filter, inputs=['user/throttle'], outputs=['user/throttle'])

    # autopilot
    class ImgPreProcess():
        '''
        preprocess camera image for inference.
        normalize and crop if needed.
        '''

        def __init__(self, cfg):
            self.cfg = cfg

        def run(self, img_arr):
            return normalize_and_crop(img_arr, self.cfg)

    if "coral" in model_type:
        inf_input = 'cam/image_array'
    else:
        inf_input = 'cam/normalized/cropped'
        V.add(ImgPreProcess(cfg),
              inputs=['cam/image_array'],
              outputs=[inf_input],
              run_condition='run_pilot')

    # Behavioral state
    if cfg.TRAIN_BEHAVIORS:
        bh = BehaviorPart(cfg.BEHAVIOR_LIST)
        V.add(bh, outputs=['behavior/state', 'behavior/label', "behavior/one_hot_state_array"])
        try:
            ctr.set_button_down_trigger('L1', bh.increment_state)
        except:
            pass

        inputs = [inf_input, "behavior/one_hot_state_array"]
        # IMU
    elif model_type == "imu":
        assert (cfg.HAVE_IMU)
        # Run the pilot if the mode is not user.
        inputs = [inf_input,
                  'imu/acl_x', 'imu/acl_y', 'imu/acl_z',
                  'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z']
    else:
        inputs = [inf_input]

    def load_model(kl, model_path):
        start = time.time()
        print('loading model', model_path)
        kl.load(model_path)
        print('finished loading in %s sec.' % (str(time.time() - start)))

    def load_weights(kl, weights_path):
        start = time.time()
        try:
            print('loading model weights', weights_path)
            kl.model.load_weights(weights_path)
            print('finished loading in %s sec.' % (str(time.time() - start)))
        except Exception as e:
            print(e)
            print('ERR>> problems loading weights', weights_path)

    def load_model_json(kl, json_fnm):
        start = time.time()
        print('loading model json', json_fnm)
        from tensorflow.python import keras
        try:
            with open(json_fnm, 'r') as handle:
                contents = handle.read()
                kl.model = keras.models.model_from_json(contents)
            print('finished loading json in %s sec.' % (str(time.time() - start)))
        except Exception as e:
            print(e)
            print("ERR>> problems loading model json", json_fnm)

    if model_path:
        # When we have a model, first create an appropriate Keras part
        kl = dk.utils.get_model_by_type(model_type, cfg)

        model_reload_cb = None

        if '.h5' in model_path or '.uff' in model_path or 'tflite' in model_path or '.pkl' in model_path:
            # when we have a .h5 extension
            # load everything from the model file
            load_model(kl, model_path)

            def reload_model(filename):
                load_model(kl, filename)

            model_reload_cb = reload_model

        elif '.json' in model_path:
            # when we have a .json extension
            # load the model from there and look for a matching
            # .wts file with just weights
            load_model_json(kl, model_path)
            weights_path = model_path.replace('.json', '.weights')
            load_weights(kl, weights_path)

            def reload_weights(filename):
                weights_path = filename.replace('.json', '.weights')
                load_weights(kl, weights_path)

            model_reload_cb = reload_weights

        else:
            print("ERR>> Unknown extension type on model file!!")
            return

        outputs = ['pilot/angle', 'pilot/throttle']

        if cfg.TRAIN_LOCALIZER:
            outputs.append("pilot/loc")

        V.add(kl, inputs=inputs,
              outputs=outputs,
              run_condition='run_pilot')

    class AiRunCondition:
        '''
        A bool part to let us know when ai is running.
        '''

        def run(self, mode):
            if mode == "user":
                return False
            return True

    V.add(AiRunCondition(), inputs=['user/mode'], outputs=['ai_running'])

    # add LIDAR stuff
    # NOTE: using part from donkeyacc
    lidar_processor = LidarProcessor(non_block=False, debug=True)  # use non-block mode for more fps
    V.add(lidar_processor, outputs=['lidar/distance'], threaded=False)

    # add cruise controller
    # NOTE: using part from donkeyacc
    cruise_controller = CruiseController(kp=1, kd=1.5, default_distance=0.5, throttle_scale=cfg.JOYSTICK_MAX_THROTTLE,
                                         debug=True)  # TODO: disable debug mode
    V.add(cruise_controller, inputs=['lidar/distance', 'user/throttle'], outputs=['acc/throttle'], threaded=False)

    # TODO: YOLO detection
    # TODO: autopilot
    # TODO: load model

    class DriveMode:
        """
        Change different drive modes: manual, adaptive CC, autopilot, etc...
        """
        def run(self, mode,
                user_angle, user_throttle,
                pilot_angle, pilot_throttle):
            # NOTE: pilot_throttle is now handled by ACC
            if mode == 'user':
                # using user angle and throttle
                return user_angle, user_throttle

            elif mode == 'local_angle':
                # using user angle and ACC throttle
                return user_angle, pilot_throttle

            else:
                # using autopilot angle and ACC throttle
                return pilot_angle, pilot_throttle

    V.add(DriveMode(),
          inputs=['user/mode', 'user/angle', 'user/throttle',
                  'pilot/angle', 'acc/throttle'],
          outputs=['angle', 'throttle'])

    # setup drive train
    steering_controller = PCA9685(
        cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM)

    throttle_controller = PCA9685(
        cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    throttle = PWMThrottle(controller=throttle_controller,
                           max_pulse=cfg.THROTTLE_FORWARD_PWM,
                           zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                           min_pulse=cfg.THROTTLE_REVERSE_PWM)

    V.add(steering, inputs=['angle'])
    V.add(throttle, inputs=['throttle'])

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
