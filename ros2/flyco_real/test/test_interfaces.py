"""Exercise geometry and the actual ROS transport without a physical flight controller."""
import math
import time
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Empty
from std_srvs.srv import SetBool, Trigger
from flyco_real.geometry import transform_matrix, world_alignment, planner_gimbal
from flyco_real.control import fresh, validate_command
from flyco_real.px4_bridge import PX4Bridge


def test_camera_and_frame_alignment():
    body = transform_matrix([3., -2., 1.], [0., 0., 0., 1.])
    optical = transform_matrix([3., -2., 1.], [-.5, .5, -.5, .5])
    q = planner_gimbal(body, optical)
    assert np.allclose(q, [0., 0., 0., 1.])
    px4 = transform_matrix([10., 20., 2.], [0., 0., math.sin(math.pi/4), math.cos(math.pi/4)])
    rotation, offset, yaw = world_alignment(body, px4)
    assert np.allclose(rotation @ body[:3, 3] + offset, px4[:3, 3])
    assert yaw == pytest.approx(math.pi / 2)


def test_invalid_commands_and_time():
    def validate(p=(0., 0., 1.), v=(0., 0., 0.), a=(0., 0., 0.)):
        return validate_command(p, v, a, 0., 0., [0., 0., 0.], [-2., -2., -1.], [2., 2., 4.], 1., 2., 1., 2.)
    assert validate() == ''
    assert 'nonfinite' in validate(p=[math.nan, 0., 0.])
    assert 'volume' in validate(p=[3., 0., 0.])
    assert 'velocity' in validate(v=[2., 0., 0.])
    assert 'acceleration' in validate(a=[0., 0., 3.])
    assert fresh(1., 1.1, .3)
    assert not fresh(2., 1., .3)  # Future packets cannot renew the watchdog.
    assert not fresh(None, 1., .3)


def test_raw_camera_rectification_preserves_optical_center():
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
    from flyco_real.perception_bridge import PerceptionBridge
    rclpy.init()
    bridge, fake = PerceptionBridge(), Node('camera_probe')
    bridge.image_is_rectified = False
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(fake)
    received = []
    fake.create_subscription(Image, '/flyco/camera/rgb', received.append, 10)
    cv = CvBridge()
    try:
        for model, distortion in [('plumb_bob', [-.1, 0., 0., 0., 0.]),
                                  ('equidistant', [0., 0., 0., 0.])]:
            stamp = fake.get_clock().now().to_msg()
            info = CameraInfo(header=Header(stamp=stamp, frame_id='camera_optical_frame'),
                              width=32, height=24, distortion_model=model, d=distortion)
            info.k = [20., 0., 16., 0., 20., 12., 0., 0., 1.]
            info.p = [18., 0., 16., 0., 0., 18., 12., 0., 0., 0., 1., 0.]
            bridge.on_info(info)
            image = np.zeros((24, 32, 3), np.uint8)
            image[12, 16] = [255, 255, 255]
            raw = cv.cv2_to_imgmsg(image, encoding='rgb8')
            raw.header = info.header
            old_count = len(received)
            deadline = time.monotonic() + 2.
            while len(received) == old_count and time.monotonic() < deadline:
                bridge.on_image(raw)
                executor.spin_once(timeout_sec=.02)
            assert len(received) > old_count
            output = cv.imgmsg_to_cv2(received[-1], desired_encoding='rgb8')
            assert np.all(output[12, 16] == 255)
            assert output.shape == image.shape
            assert bridge.camera_info.k[0] == 18.
            assert not any(bridge.camera_info.d)
    finally:
        executor.remove_node(fake)
        executor.remove_node(bridge)
        fake.destroy_node()
        bridge.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
