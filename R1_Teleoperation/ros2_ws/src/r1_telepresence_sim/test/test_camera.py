import numpy as np
import pytest
from sensor_msgs.msg import Image

from r1_telepresence_sim.camera_viewer import StereoCameraViewer


def _image(encoding, width, height, data, step):
    message = Image()
    message.encoding = encoding
    message.width = width
    message.height = height
    message.step = step
    message.data = list(data)
    return message


def test_rgb8_and_row_padding_are_converted_to_bgr():
    # RGB pixels: red, green; each row has two padding bytes.
    message = _image('rgb8', 2, 1, [255, 0, 0, 0, 255, 0, 9, 9], 8)
    output = StereoCameraViewer._image_to_bgr(message)
    assert output.tolist() == [[[0, 0, 255], [0, 255, 0]]]


def test_mono8_is_expanded_for_stereo_concat():
    message = _image('mono8', 2, 1, [12, 240], 2)
    output = StereoCameraViewer._image_to_bgr(message)
    assert output.shape == (1, 2, 3)
    assert np.array_equal(output[0, 0], [12, 12, 12])
    assert np.array_equal(output[0, 1], [240, 240, 240])


def test_unsupported_encoding_is_rejected():
    message = _image('16UC1', 1, 1, [0, 0], 2)
    with pytest.raises(ValueError, match='unsupported image encoding'):
        StereoCameraViewer._image_to_bgr(message)
