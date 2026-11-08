"""Rectify Odin's native polynomial lens outside the unmodified vendor driver."""
import numpy as np
from sensor_msgs.msg import CameraInfo


def camera_calibration(document, frame):
    cam = document['cam_0']
    width, height = int(cam['image_width']), int(cam['image_height'])
    fx, fy, cx, cy, skew = [float(cam[k]) for k in ('A11', 'A22', 'u0', 'v0', 'A12')]
    coefficients = np.array([float(cam['k%d' % n]) for n in range(2, 8)])
    if (width <= 0 or height <= 0 or fx <= 0. or fy <= 0. or
            not np.isfinite([fx, fy, cx, cy, skew, *coefficients]).all()):
        raise ValueError('Invalid Odin camera calibration')
    # Output is a zero-skew pinhole camera. The polynomial maps its rays back
    # into the native distorted image; these coefficients are not OpenCV D.
    v, u = np.indices((height, width), dtype=np.float64)
    x, y = (u - cx) / fx, (v - cy) / fy
    radius = np.hypot(x, y)
    theta = np.arctan(radius)
    distorted = theta.copy()
    for power, coefficient in enumerate(coefficients, start=2):
        distorted += coefficient * theta ** power
    if np.any(np.abs(coefficients) > 1.e-7):
        scale = np.divide(distorted, radius, out=np.ones_like(radius), where=radius > 1.e-12)
        x, y = x * scale, y * scale
        map_x = fx * x + skew * y + cx
    else:
        # Match the native model's undistorted world2cam branch.
        map_x = fx * x + cx
    map_y = fy * y + cy
    info = CameraInfo(width=width, height=height, distortion_model='plumb_bob', d=[0.] * 5)
    info.header.frame_id = frame
    info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    info.r = np.eye(3).flatten().tolist()
    info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
    return info, (map_x.astype(np.float32), map_y.astype(np.float32))


def image_array(msg):
    """View native BGR/RGB bytes without crossing the cv_bridge C++ ABI."""
    if msg.encoding not in ('bgr8', 'rgb8') or msg.step < msg.width * 3:
        raise ValueError('Expected a BGR8/RGB8 image with a valid row stride')
    if len(msg.data) < msg.height * msg.step:
        raise ValueError('Image payload is shorter than its dimensions')
    return np.ndarray((msg.height, msg.width, 3), dtype=np.uint8,
                      buffer=msg.data, strides=(msg.step, 3, 1))
