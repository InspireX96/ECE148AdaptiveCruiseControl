"""
Unit tests for YOLO Processor
"""

import time
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


@pytest.mark.skip(reason='Do not test YOLO automatically')
def test_block_mode():
    """
    YOLO Processor block mode
    """
    print('\n===== YOLO Processor block mode test =====\n')
    processor = YoloProcessor(use_tiny_yolo=True, non_block=False, debug=True)

    try:
        img_array = np.array(Image.open((os.getcwd() + '/dataset/stop_sign,jpg')))
    except FileNotFoundError:
        img_array = np.array(Image.open((os.getcwd() + '/tests/dataset/stop_sign.jpg')))

    output_list = []
    time_start = time.time()
    for i in range(2):
        output = processor.run(img_array)
        print('output {}: {}'.format(i, output))
        output_list.append(output)
    elapsed_time = time.time() - time_start
    print(output_list)
    print('Elapsed time: ', elapsed_time)
    assert elapsed_time > 0.1


@pytest.mark.skip(reason='Do not test YOLO automatically')
def test_non_block_mode():
    """
    YOLO Processor non block mode
    """
    print('\n===== YOLO Processor non block mode test =====\n')
    processor = YoloProcessor(use_tiny_yolo=True, non_block=True, debug=True)

    try:
        img_array = np.array(Image.open((os.getcwd() + '/dataset/stop_sign,jpg')))
    except FileNotFoundError:
        img_array = np.array(Image.open((os.getcwd() + '/tests/dataset/stop_sign.jpg')))

    output_list = []
    time_start = time.time()
    for i in range(100):
        output = processor.run(img_array)
        print('output {}: {}'.format(i, output))
        output_list.append(output)
        time.sleep(0.05)
    elapsed_time = time.time() - time_start
    print(output_list)
    print('Elapsed time: ', elapsed_time)
    assert elapsed_time < 10


if __name__ == '__main__':
    print('Testing YOLO Processor')
    test_save_and_find_image()
    test_integration_test()
    test_block_mode()
    test_non_block_mode()
    print('All testes passed')
