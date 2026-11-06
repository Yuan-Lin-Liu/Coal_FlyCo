"""Odin live-calibration transport, body origin and time/rate normalization."""
import copy
import time
import numpy as np
import pytest
import rclpy
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from flyco_real.odin_adapter import OdinAdapter, sdk_extrinsic
from flyco_real.sensor_bridge import SensorBridge
from flyco_real.geometry import transform_matrix, tf_matrix


def test_wiwc_rejects_invalid_rigid_calibration():
    for matrix in (np.zeros((4, 4)), np.diag([2., 1., 1., 1.]),
                   np.diag([-1., 1., 1., 1.]), np.full((4, 4), np.nan)):
        with pytest.raises(ValueError): sdk_extrinsic(matrix.flatten())
    assert np.array_equal(sdk_extrinsic(np.eye(4).flatten()), np.eye(4))


@pytest.mark.parametrize('send_vision',[True,False])
def test_odin_late_calibration_and_sensor_pipeline(tmp_path,send_vision):
    config = yaml.safe_load((Path(get_package_share_directory('flyco_real')) / 'config/odin.yaml').read_text())
    config = {k:v for k,v in config.items() if isinstance(v,dict) and 'ros__parameters' in v}
    config['odin_adapter']['ros__parameters']['body_to_imu_translation'] = [.2,0.,0.]
    config['sensor_bridge']['ros__parameters']['publish_external_vision'] = send_vision
    profile = tmp_path / 'parameters.yaml'
    profile.write_text(yaml.safe_dump(config))
    rclpy.init(args=['--ros-args','--params-file',str(profile)])
    fake = Node('odin_transport_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    calibration_pub = fake.create_publisher(Odometry, '/odin1/wiwc', retained)
    imu_lidar = transform_matrix([.05,0.,0.],[0.,0.,0.,1.])
    imu_camera = transform_matrix([.1,0.,0.],[-.5,.5,-.5,.5])
    wi = Odometry()
    wi.pose.covariance = (np.linalg.inv(imu_camera) @ imu_lidar).flatten().tolist() + [0.]*20
    wi.twist.covariance = imu_lidar.flatten().tolist() + [0.]*20
    calibration_pub.publish(wi)  # Sent BEFORE subscriber creation.
    adapter, bridge = OdinAdapter(), SensorBridge()
    executor.add_node(adapter); executor.add_node(bridge)
    buffer=Buffer(node=fake); listener=TransformListener(buffer,fake)
    outputs, clouds, images, infos, visions = [], [], [], [], []
    fake.create_subscription(Odometry,'/flyco/odom',outputs.append,10)
    fake.create_subscription(PointCloud2,'/flyco/lidar/world',clouds.append,10)
    fake.create_subscription(Image,'/flyco/camera/rgb',images.append,10)
    fake.create_subscription(CameraInfo,'/flyco/camera/info',infos.append,10)
    fake.create_subscription(VehicleOdometry,'/fmu/in/vehicle_visual_odometry',visions.append,10)
    odom_pub=fake.create_publisher(Odometry,'/odin1/odometry_highfreq',qos_profile_sensor_data)
    cloud_pub=fake.create_publisher(PointCloud2,'/odin1/cloud_slam',qos_profile_sensor_data)
    image_pub=fake.create_publisher(Image,'/odin1/image/undistorted',qos_profile_sensor_data)
    info_pub=fake.create_publisher(CameraInfo,'/odin1/camera_info',qos_profile_sensor_data)
    live={'active':True, 'inputs':0}
    def publish():
        if not live['active']: return
        stamp=fake.get_clock().now().to_msg()
        msg=Odometry(header=Header(stamp=stamp,frame_id='odom'),child_frame_id='imu')
        msg.pose.pose.position.x, msg.pose.pose.position.z, msg.pose.pose.orientation.w = 1.2, 1., 1.
        odom_pub.publish(msg); live['inputs']+=1
        if live['inputs'] % 10: return
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(msg.header,[[3.3,0.,1.]]))
        header=Header(stamp=stamp,frame_id='odin_camera_optical_frame')
        info=CameraInfo(header=header,width=32,height=24,distortion_model='plumb_bob',d=[0.]*5)
        info.k=[20.,0.,16.,0.,20.,12.,0.,0.,1.]
        info.r=np.eye(3).flatten().tolist()
        info.p=[20.,0.,16.,0.,0.,20.,12.,0.,0.,0.,1.,0.]
        info_pub.publish(info)
        image_pub.publish(Image(header=header,width=32,height=24,encoding='bgr8',step=96,data=bytes(32*24*3)))
    timer=fake.create_timer(.005,publish)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until: executor.spin_once(timeout_sec=.005)
    try:
        spin(2.)
        assert adapter.calibration is not None, 'Retained WIWC must reach a late subscriber'
        assert outputs and clouds and images and infos
        assert bool(visions) == send_vision
        assert outputs[-1].pose.pose.position.x == pytest.approx(1.)
        if send_vision: assert np.allclose(visions[-1].position,[0.,1.,-1.])
        assert len(outputs) < live['inputs']*.6
        stamps=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in outputs]
        assert np.min(np.diff(stamps)) >= .0199
        camera=tf_matrix(buffer.lookup_transform('world','odin_camera_optical_frame',Time()).transform)
        assert camera[0,3] == pytest.approx(1.3)
        optical=np.linalg.inv(camera) @ [3.3,0.,1.,1.]
        assert np.allclose(optical[:3],[0.,0.,2.])
        assert infos[-1].k[0] == 20. and not any(infos[-1].d)
        assert images[-1].header.frame_id == 'odin_camera_optical_frame'
        assert np.allclose(point_cloud2.read_points(clouds[-1],field_names=('x','y','z')).tolist(),[[3.3,0.,1.]])
        # Stale device-boot timestamps cannot become fresh external-vision data.
        live['active']=False; spin(.1); before=len(outputs)
        stale=copy.deepcopy(outputs[-1]); stale.header.frame_id='odom'; stale.child_frame_id='imu'
        stale.header.stamp.sec=0; stale.header.stamp.nanosec=0
        odom_pub.publish(stale); spin(.1)
        assert len(outputs)==before
        changed=copy.deepcopy(wi)
        changed.pose.covariance[3] += .1
        with pytest.raises(RuntimeError,match='calibration changed'):
            adapter.on_extrinsics(changed)
    finally:
        for node in (bridge,adapter,fake): executor.remove_node(node); node.destroy_node()
        executor.shutdown(); rclpy.shutdown()
