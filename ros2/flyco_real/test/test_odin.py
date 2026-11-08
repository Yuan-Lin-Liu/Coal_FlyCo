"""Exercise the imported C++ Odin forwarder and the separate perception process."""
import copy
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from flyco_real.geometry import tf_matrix


def test_cpp_odin_forwarding_and_perception(tmp_path):
    share = Path(get_package_share_directory('flyco_real'))
    profile = yaml.safe_load((share / 'config/odin.yaml').read_text())
    config = tmp_path / 'vehicle.yaml'
    config.write_text(yaml.safe_dump(profile))
    log_path = tmp_path / 'launch.log'
    rclpy.init()
    fake = Node('odin_cpp_pipeline_probe')
    outputs, visions, clouds, images, infos, feedback = [], [], [], [], [], []
    fake.create_subscription(Odometry, '/flyco/odom', outputs.append, 10)
    fake.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', visions.append, 10)
    fake.create_subscription(PointCloud2, '/flyco/lidar/world', clouds.append, 10)
    fake.create_subscription(Image, '/flyco/camera/rgb', images.append, 10)
    fake.create_subscription(CameraInfo, '/flyco/camera/info', infos.append, 10)
    fake.create_subscription(QuaternionStamped, '/flyco/gimbal_pose', feedback.append, 10)
    odom = fake.create_publisher(Odometry, '/odin1/odometry_highfreq', qos_profile_sensor_data)
    cloud = fake.create_publisher(PointCloud2, '/odin1/cloud_slam', qos_profile_sensor_data)
    image = fake.create_publisher(Image, '/odin1/image/undistorted', qos_profile_sensor_data)
    info = fake.create_publisher(CameraInfo, '/odin1/camera_info', qos_profile_sensor_data)
    broadcaster = TransformBroadcaster(fake)
    buffer = Buffer(node=fake)
    listener = TransformListener(buffer, fake)
    live = dict(active=True, camera_tf=False, count=0)

    def publish():
        if not live['active']: return
        stamp = fake.get_clock().now().to_msg()
        msg = Odometry(header=Header(stamp=stamp, frame_id='odom'), child_frame_id='imu')
        msg.pose.pose.position.x, msg.pose.pose.position.z = 1.2, 1.
        msg.pose.pose.orientation.w = 1.
        # Odin's compact high-frequency packets have no covariance or velocity.
        odom.publish(msg)
        live['count'] += 1
        if live['count'] % 10: return
        if live['camera_tf']:
            imu = TransformStamped(header=msg.header, child_frame_id='imu')
            imu.transform.translation.x, imu.transform.translation.z, imu.transform.rotation.w = 1.2, 1., 1.
            lidar = TransformStamped(header=Header(stamp=stamp, frame_id='imu'), child_frame_id='lidar')
            lidar.transform.translation.x, lidar.transform.rotation.w = .05, 1.
            camera = TransformStamped(header=msg.header, child_frame_id='camera_0')
            camera.transform.translation.x, camera.transform.translation.z = 1.3, 1.
            camera.transform.rotation.x, camera.transform.rotation.y = -.5, .5
            camera.transform.rotation.z, camera.transform.rotation.w = -.5, .5
            broadcaster.sendTransform([imu, lidar, camera])
        cloud.publish(point_cloud2.create_cloud_xyz32(msg.header, [[3.3, 0., 1.]]))
        header = Header(stamp=stamp, frame_id='odin_camera_optical_frame')
        calibration = CameraInfo(header=header, width=32, height=24, distortion_model='plumb_bob', d=[0.]*5)
        calibration.k = [20., 0., 16., 0., 20., 12., 0., 0., 1.]
        calibration.r = np.eye(3).flatten().tolist()
        calibration.p = [20., 0., 16., 0., 0., 20., 12., 0., 0., 0., 1., 0.]
        info.publish(calibration)
        image.publish(Image(header=header, width=32, height=24, encoding='bgr8', step=96, data=bytes(32*24*3)))

    fake.create_timer(.01, publish)
    log = log_path.open('w')
    process = subprocess.Popen(['ros2', 'launch', 'flyco_real', 'hardware.launch.py',
        'role:=sensors', 'start_driver:=false', 'config:='+str(config),
        'runtime_root:='+str(tmp_path/'runtime')], cwd='/tmp', stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)

    def spin(seconds, until=lambda: False):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline and not until():
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(fake, timeout_sec=.01)

    try:
        spin(10., lambda: len(outputs) >= 8 and len(visions) >= 8)
        assert outputs and visions, log_path.read_text()
        assert not live['camera_tf']  # PX4 must work before native camera TF is ready.
        live['camera_tf'] = True
        spin(5., lambda: bool(clouds and images and infos and feedback))
        assert clouds and images and infos and feedback, log_path.read_text()
        assert outputs[-1].header.frame_id == 'world'
        assert outputs[-1].child_frame_id == 'base_link'
        assert outputs[-1].pose.pose.position.x == 1.2
        np.testing.assert_allclose(visions[-1].position, [0., 1.2, -1.], atol=1.e-6)
        np.testing.assert_allclose(np.abs(visions[-1].q), [math.sqrt(.5), 0., 0., math.sqrt(.5)], atol=1.e-6)
        assert visions[-1].pose_frame == VehicleOdometry.POSE_FRAME_FRD
        assert visions[-1].velocity_frame == VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        assert np.isnan(visions[-1].velocity).all()
        np.testing.assert_allclose(visions[-1].position_variance, [.04, .04, .09])
        stamps = {Time.from_msg(m.header.stamp).nanoseconds//1000 for m in outputs}
        assert any(m.timestamp_sample in stamps for m in visions)
        assert images[-1].header.frame_id == infos[-1].header.frame_id == 'camera_0'
        assert tf_matrix(buffer.lookup_transform('world', 'base_link', Time()).transform)[0, 3] == 1.2
        for topic in ('/flyco/odom', '/fmu/in/vehicle_visual_odometry'):
            assert {entry.node_name for entry in fake.get_publishers_info_by_topic(topic)} == {'generic_odometry_node'}
        live['active'] = False
        spin(.2)
        before = (len(outputs), len(visions))
        invalid = copy.deepcopy(outputs[-1])
        invalid.header.frame_id, invalid.child_frame_id = 'odom', 'imu'
        invalid.header.stamp.sec = invalid.header.stamp.nanosec = 0
        odom.publish(invalid)
        spin(.15)
        invalid.header.stamp = fake.get_clock().now().to_msg()
        invalid.pose.pose.orientation.w = 0.
        odom.publish(invalid)
        spin(.15)
        assert (len(outputs), len(visions)) == before
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        fake.destroy_node()
        rclpy.shutdown()
