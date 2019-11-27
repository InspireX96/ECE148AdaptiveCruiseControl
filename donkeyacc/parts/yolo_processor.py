"""
Interface between donkey framework and YOLO
Before running this script, please ensure that Darknet YOLO is installed in ~/projects/d3/
Please refer to README.md for installation guide
"""

import os
import logging
import time
import threading
from multiprocessing.pool import ThreadPool
import glob
import shutil
import subprocess
from PIL import Image
import numpy as np


class YoloProcessor(object):
    """
    YOLO Processor as donkey part

    This processor will get an input image from donkey framework, save it in a temp folder and user YOLO to detect
    """

    def __init__(self, use_tiny_yolo=True, non_block=True, warning=True, debug=False):
        """
        Constructor of YOLOProcessor
        :param use_tiny_yolo: bool, flag to use tiny YOLO
        :param non_block: bool, flag to turn on non-block mode.
                          If non_block=False, the processor will block the process while waiting for YOLO.
                              Once YOLO detection is completed, it will return back.
                          If non_block=True, the processor will send back previous YOLO detection result while
                              waiting for a new YOLO detection, hence it will not block the process.
        :param warning: bool, flag to turn on warning to user if person/stop sign etc is detected
        :param debug: bool, flag to turn on debug mode that prints out YOLO detection result
        """
        self.use_tiny_yolo = use_tiny_yolo
        self.non_block = non_block
        self.warning = warning
        self.debug = debug
        print('Initializing YOLO Processor, use_tiny_yolo: {}'.format(use_tiny_yolo))

        self.project_path = os.path.join(os.path.expanduser('~'), 'projects/d3')
        self.darknet_path = os.path.join(self.project_path, 'darknet')

        # test darknet installation
        try:
            subprocess.run(os.path.join(self.darknet_path, 'darknet'))
        except Exception as err:
            raise Exception('Cannot run darknet, please make sure it is installed in ~/projects/d3\n', err)

        # create temp folder to save image
        self.yolo_data_path = os.path.join(self.project_path, 'yolo_data')
        if os.path.isdir(self.yolo_data_path):
            print('Deleting existing yolo_data folder')
            shutil.rmtree(self.yolo_data_path)
        print('Making folder: {}'.format(self.yolo_data_path))
        os.mkdir(self.yolo_data_path)

        self.result = {}
        self.counter = 0
        # flag
        self.is_running = False

    def _save_image(self, image):
        """
        Save input image to yolo_data
        :param image: np ndarray, image
        """
        img = Image.fromarray(np.uint8(image))
        img_name = str(self.counter) + '.jpg'
        img.save(os.path.join(self.yolo_data_path, img_name))
        self.counter += 1

    def _find_latest_image_name(self):
        """
        Find latest image path in yolo_data
        :return: str, absolute path of latest image
        """

        image_name = max(glob.iglob(os.path.join(self.yolo_data_path, '*.jpg')), key=os.path.getctime)
        if self.debug:
            print('Found latest image: {}'.format(image_name))
        return image_name

    @staticmethod
    def _parse_yolo_output(yolo_output):
        """
        Parse YOLO output, save detected objects in a dictionary
        :param yolo_output: str, stdout of yolo.
        :return: dict, keys are detected objects (str),
                       values are corresponding confidence (float, 0~1)
        """
        output = yolo_output.strip().split('\n')[1:]
        result = {}
        for item in output:
            key, value = item.split(':')
            key = key.strip()
            value = int(''.join(filter(str.isdigit, value))) / 100
            if key not in result.keys():
                result[key] = value

        return result

    def _run_yolo(self):
        """
        Run YOLO to detect latest image in yolo_data
        :return: str, stdout of yolo.
                 Example: '<image>: Predicted in 0.617665 seconds.\nstop sign: 100%\nstop sign: 55%\n'
        """
        # change path to solve YOLO path problem
        old_path = os.getcwd()
        os.chdir(self.darknet_path)

        image_name = self._find_latest_image_name()  # find latest image

        if self.use_tiny_yolo:
            yolo_cfg = 'cfg/yolov3-tiny.cfg'
            yolo_weights = 'yolov3-tiny.weights'
        else:
            yolo_cfg = 'cfg/yolov3.cfg'
            yolo_weights = 'yolov3.weights'

        command = ['./darknet', 'detect', yolo_cfg, yolo_weights, image_name]
        output = subprocess.check_output(command, stderr=subprocess.DEVNULL)

        if self.debug:
            print('YOLO output: {}'.format(output))

        os.chdir(old_path)  # change back working dir

        # parse result
        self.result = self._parse_yolo_output(output.decode())

        self.is_running = False

    def run(self, image):
        """
        :param image: np ndarray, image
        :return: dict, keys are detected objects (str),
                       values are corresponding confidence (float, 0~1)
        """
        # TODO: multi thread and non blocking
        if not image:
            logging.warning('No input image to YOLO processor')
            return self.result
        if not self.is_running:
            self.is_running = True
            self._save_image(image)  # save img

            pool = ThreadPool(processes=1)
            t = threading.Thread(target=self._run_yolo)  # run YOLO
            t.start()

            if self.debug:
                print('YOLO result: {}'.format(self.result))
            if self.warning:
                if 'stop sign' in self.result.keys():
                    logging.warning('STOP SIGN Detected!')
                if 'person' in self.result.keys():
                    logging.warning('PERSON Detected!')

        if self.non_block:
            return self.result
        else:
            while self.is_running:
                time.sleep(0.02)

        return self.result
