"""
Unit tests for YOLO Processor
"""

import os
import pytest
import numpy as np
from PIL import Image
from donkeyacc.parts.yolo_processor import YoloProcessor


@pytest.mark.skip(reason='Do not test YOLO automatically')
def test_save_and_find_image():
    """
    Test save and find image
    """
    print('\n===== Testing saving and finding image =====\n')
    processor = YoloProcessor(non_block=False, debug=True)

    test_image_list = ['person.jpg', 'stop_sign.jpg']
    for test_image in test_image_list:
        try:
            img_array = np.array(Image.open((os.getcwd() + '/dataset/' + test_image)))
        except FileNotFoundError:
            img_array = np.array(Image.open((os.getcwd() + '/tests/dataset/' + test_image)))
        processor._save_image(img_array)

        # test find latest image name
        name = processor._find_latest_image_name()
        assert str(processor.counter - 1) + '.jpg' in name
    assert processor.counter == len(test_image_list)


@pytest.mark.skip(reason='Do not test YOLO automatically')
def test_run_yolo():
    """
    Test run YOLO
    """
    print('\n===== Testing running YOLO =====\n')
    processor = YoloProcessor(non_block=False, debug=True)

    try:
        img_array = np.array(Image.open((os.getcwd() + '/dataset/stop_sign,jpg')))
    except FileNotFoundError:
        img_array = np.array(Image.open((os.getcwd() + '/tests/dataset/stop_sign.jpg')))
    processor._save_image(img_array)

    # test YOLO
    output = processor._run_yolo().strip().split('\n')[1:]      # parse YOLO output
    detected = False
    for item in output:
        if 'stop sign' in item:
            detected = True
    assert detected


@pytest.mark.skip(reason='Do not test YOLO automatically')
def test_integration_test():
    """
    YOLO Processor integration test
    """
    print('\n===== YOLO Processor integration test =====\n')
    processor = YoloProcessor(non_block=False, debug=True)

    try:
        img_array = np.array(Image.open((os.getcwd() + '/dataset/stop_sign,jpg')))
    except FileNotFoundError:
        img_array = np.array(Image.open((os.getcwd() + '/tests/dataset/stop_sign.jpg')))

    # test YOLO
    output = processor.run(img_array)
    assert 'stop sign' in output.keys()


if __name__ == '__main__':
    print('Testing YOLO Processor')
    test_save_and_find_image()
    test_run_yolo()
    test_integration_test()
    print('All testes passed')
