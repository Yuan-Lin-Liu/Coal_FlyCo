"""ROS2 sensor and command adapter for the real FlyCo nodes, without AirSim."""
from array import array
import json
import os
import signal
from pathlib import Path
import socket
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from builtin_interfaces.msg import Time
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, Imu, Joy
from std_msgs.msg import Float32, Bool, String
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
from mavros_msgs.msg import AttitudeTarget
from airsim_ros_pkgs.msg import GimbalAngleEulerCmd
from quadrotor_msgs.msg import PositionCommand, PolynomialTrajGroup, Mesh
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wire import send_packet, recv_packet
from geometry import rgbd_points

RUNTIME = Path(os.environ['FLYCO_ISAAC_RUNTIME'])


def stamp(t):
    ns = round(t*1e9)
    return Time(sec=ns//10**9, nanosec=ns % 10**9)


class Bridge(Node):
    def __init__(self):
        super().__init__('isaac_bridge')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.clock = self.create_publisher(Clock, '/clock', 10)
        self.odom = self.create_publisher(Odometry, '/isaac/odom', qos)
        self.imu = self.create_publisher(Imu, '/isaac/imu', qos)
        self.cloud = self.create_publisher(PointCloud2, '/isaac/lidar/world', qos)
        self.rgb = self.create_publisher(Image, '/isaac/camera/rgb', qos)
        self.depth = self.create_publisher(Image, '/isaac/camera/depth', qos)
        self.info = self.create_publisher(CameraInfo, '/isaac/camera/info', qos)
        self.rgbcloud = self.create_publisher(PointCloud2, '/isaac/camera/world_points', qos)
        self.gimbal = self.create_publisher(QuaternionStamped, '/gimbal_pose', qos)
        self.joy = self.create_publisher(Joy, '/joy', 10)
        self.query_text = self.create_publisher(String, '/query_text', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        tr = TransformStamped()
        tr.header.frame_id = 'drone_1/odom_local_enu'
        tr.child_frame_id = 'front_center_body/static'
        tr.transform.translation.x = .125
        tr.transform.rotation.w = 1.
        self.static_tf.sendTransform(tr)
        self.lock = threading.Lock()
        self.control = None
        self.position_command = None
        self.position_count = 0
        self.trajectory_windows = []
        self.gimbal_command = [0., 0., 0.]
        self.intent = 'GROUNDED'
        self.generation = 0
        self.control_count = 0
        self.action_sequence = 0
        self.telemetry = {'state_frames': 0, 'cloud_frames': 0, 'image_frames': 0,
                          'mesh_messages': 0, 'coverage': None, 'flight_end': False}
        self.last_t = 0.
        self.last_wall = 0.
        self.stop = threading.Event()
        self.pulses = []
        self.native_controls = {}
        if os.environ.get('FLYCO_ISAAC_FLIGHT_CONTROLLER') == 'px4-native':
            self.native_controls = {action: self.create_client(Trigger, '/flyco/control/'+service)
                for action, service in (('initial', 'takeoff'), ('hold', 'hold'), ('resume', 'resume'))}
        self.create_subscription(AttitudeTarget, '/setpoint_raw/attitude', self.on_control, qos)
        self.create_subscription(GimbalAngleEulerCmd, '/gimbal_angle_euler_cmd',
                                 self.on_gimbal, qos)
        self.create_subscription(PositionCommand, '/planning/pos_cmd', self.on_position, qos)
        self.create_subscription(PolynomialTrajGroup, '/local_newest_traj', self.on_trajectory, qos)
        self.create_subscription(Mesh, '/prediction/predicted_mesh', self.on_mesh, qos)
        self.create_subscription(PointCloud2, '/tracking/target_pc',
            lambda msg: self.telemetry.update(target_cloud_points=msg.width*msg.height), qos)
        self.create_subscription(PointCloud2, '/mesh_pred/pred_pc',
            lambda msg: self.telemetry.update(predicted_cloud_points=msg.width*msg.height), qos)
        self.create_subscription(Float32, '/planning/flight_coverage',
                                 lambda msg: self.telemetry.update(coverage=msg.data), qos)
        self.create_subscription(Bool, '/flight_end',
                                 lambda msg: self.telemetry.update(flight_end=msg.data), qos)
        self.create_timer(.05, self.actions)
        self.worker = threading.Thread(target=self.serve, daemon=True)
        self.worker.start()

    def on_control(self, msg):
        values = [msg.body_rate.x, msg.body_rate.y, msg.body_rate.z, msg.thrust]
        if not np.all(np.isfinite(values)):
            return
        with self.lock:
            self.control_count += 1
            self.control = {'count': self.control_count, 'rpy_rate': values[:3],
                            'throttle': values[3]}

    def on_position(self, msg):
        self.position_count += 1
        with self.lock:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            # The legacy traj_server does not update PositionCommand's ID.
            # Match its timestamp against the planner's polynomial schedule.
            active = [w for w in self.trajectory_windows if w['start'] <= t]
            window = max(active, key=lambda w: w['start']) if active else None
            self.position_command = {
                'p': [msg.position.x, msg.position.y, msg.position.z],
                'v': [msg.velocity.x, msg.velocity.y, msg.velocity.z],
                'a': [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
                'yaw': msg.yaw, 'yaw_dot': msg.yaw_dot,
                'count': self.position_count, 't': t,
                'trajectory_id': window['id'] if window else msg.trajectory_id,
                'trajectory_end': window['end'] if window else None}

    def on_trajectory(self, msg):
        if msg.action != PolynomialTrajGroup.ACTION_ADD or not msg.pos_traj:
            return
        start = msg.start_time.sec + msg.start_time.nanosec*1e-9
        with self.lock:
            self.trajectory_windows = (self.trajectory_windows + [
                {'id': msg.trajectory_id, 'start': start,
                 'end': start + sum(piece.duration for piece in msg.pos_traj)}])[-8:]

    def on_mesh(self, msg):
        self.telemetry['mesh_messages'] += 1
        self.telemetry.update(mesh_vertices=msg.vertex_num, mesh_faces=msg.face_num)

    def on_gimbal(self, msg):
        with self.lock:
            self.gimbal_command = [msg.roll, msg.pitch, msg.yaw]

    def action(self, action):
        if action not in ('initialize', 'initial', 'plan', 'scan', 'hold', 'land', 'resume', 'disarm'):
            raise ValueError('Unknown action: '+str(action))
        if action == 'initialize' and (self.last_t < 3. or self.odom.get_subscription_count() < 3):
            raise ValueError('Wait for FlyCo body/gimbal subscribers before Initialize')
        if action == 'initial' and not self.ready_for_initial():
            raise ValueError('Wait for FlyCo initialization and initial-trajectory publisher before Initial flight')
        if action in self.native_controls:
            client = self.native_controls[action]
            if not client.service_is_ready():
                raise ValueError('Wait for the native FlyCo control service')
            future = client.call_async(Trigger.Request())
            def completed(result):
                try:
                    response = result.result()
                    self.telemetry['native_control_result'] = {
                        'action': action, 'success': response.success, 'message': response.message}
                    if not response.success:
                        self.get_logger().warning('[ACTION] '+response.message)
                except Exception as exc:
                    self.telemetry['native_control_result'] = {
                        'action': action, 'success': False, 'message': str(exc)}
            future.add_done_callback(completed)
        if action in ('initialize', 'initial', 'hold', 'land', 'resume', 'disarm'):
            self.intent = action
            self.generation += 1
        if action in ('initialize', 'initial', 'plan', 'scan') and action not in self.native_controls:
            button = {'initialize': 4, 'initial': 3, 'plan': 2, 'scan': 5}[action]
            self.publish_joy(button)
            self.pulses.append(time.monotonic()+.2)

    def ready_for_initial(self):
        # Native takeoff uses the executor; the legacy profile also needs
        # Init_Traj to be ready before its initial-flight Joy pulse.
        expected = 1 if os.environ.get('FLYCO_ISAAC_FLIGHT_CONTROLLER') == 'px4-native' else 2
        return self.count_publishers('/local_newest_traj') >= expected

    def handle_action(self, action, text=None):
        """Report bridge receipt separately from FlyCo execution or completion."""
        messages = {
            'initialize': '1 Initialize received; FlyCo will capture the start pose.',
            'initial': '2 Initial flight received; waiting for FlyCo trajectory/control.',
            'plan': '3 Planner activation requested. Use 4 to start scanning.',
            'scan': '4 Scan trigger sent; waiting for FlyCo planning output.',
            'hold': 'Hold requested; check flight mode and vehicle motion.',
            'land': 'Land requested; wait for landing_complete (PX4) or GROUNDED and stopped motors.',
            'resume': 'Resume requested; waiting for fresh FlyCo control.',
            'disarm': 'Disarm requested; allowed only near the ground.',
            'text': 'Target description sent to perception.',
        }
        try:
            if action == 'text':
                if not isinstance(text, str) or not text.strip():
                    raise ValueError('Target description must contain text')
                if not self.query_text.get_subscription_count():
                    raise ValueError('Wait for perception subscribers before sending text')
                self.query_text.publish(String(data=text.strip()))
            else:
                self.action(action)
            state, message = 'received', messages[action]
        except ValueError as exc:
            state, message = 'rejected', str(exc)
        self.action_sequence += 1
        event = {'sequence': self.action_sequence, 'action': str(action), 'state': state,
                 'message': message, 'sim_time': self.last_t, 'wall_epoch': time.time()}
        self.telemetry['last_action'] = event
        history = self.telemetry.setdefault('action_history', [])
        self.telemetry['action_history'] = (history+[event])[-4:]
        with (RUNTIME/'operator_events.jsonl').open('a') as stream:
            stream.write(json.dumps(event)+'\n')
        if state == 'rejected':
            self.get_logger().warning('[ACTION] rejected '+str(action)+': '+message)
        else:
            self.get_logger().info('[ACTION] received '+str(action)+': '+message)

    def publish_joy(self, button=None):
        msg = Joy()
        msg.header.stamp = stamp(self.last_t)
        msg.axes = [0., 0., 1., 0., 0., 1., 0., 0.]
        msg.buttons = [0]*11
        if button is not None:
            msg.buttons[button] = 1
        self.joy.publish(msg)

    def actions(self):
        if self.pulses and min(self.pulses) <= time.monotonic():
            self.publish_joy()
            self.pulses = [p for p in self.pulses if p > time.monotonic()]
        for path in sorted((RUNTIME/'actions').glob('*.json')):
            try:
                event = json.loads(path.read_text())
                self.handle_action(event['action'], event.get('text'))
            except (OSError, ValueError, KeyError) as exc:
                self.get_logger().error(str(exc))
            path.unlink(missing_ok=True)
        value = dict(self.telemetry, sim_time=self.last_t, control_count=self.control_count,
                     position_command=self.position_command, intent=self.intent,
                     perception_subscribers=self.query_text.get_subscription_count(),
                     ready_for_initial=self.ready_for_initial(),
                     ready_for_initialize=self.last_t >= 3. and self.odom.get_subscription_count() >= 3,
                     sensor_age_s=time.monotonic()-self.last_wall if self.last_wall else None)
        tmp = RUNTIME/'bridge_status.tmp'
        tmp.write_text(json.dumps(value, indent=2))
        tmp.replace(RUNTIME/'bridge_status.json')

    @staticmethod
    def pointcloud(points, t, colors=None):
        msg = PointCloud2()
        msg.header.stamp = stamp(t)
        msg.header.frame_id = 'world'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [PointField(name=n, offset=i*4, datatype=7, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        if colors is None:
            data = np.asarray(points, dtype='<f4')
            msg.point_step = 12
        else:
            packed = (colors[:, 0].astype('<u4') << 16 |
                      colors[:, 1].astype('<u4') << 8 | colors[:, 2].astype('<u4'))
            data = np.empty((len(points), 4), dtype='<f4')
            data[:, :3] = points
            data[:, 3] = packed.view('<f4')
            msg.fields.append(PointField(name='rgb', offset=12, datatype=7, count=1))
            msg.point_step = 16
        msg.row_step = msg.width*msg.point_step
        msg.is_dense = True
        msg.data = array('B', data.tobytes())
        return msg

    def publish(self, frame, arrays):
        t = frame['t']
        if t < self.last_t:
            raise ValueError('Simulation time reset: restart the complete session')
        self.last_t, self.last_wall = t, time.monotonic()
        self.clock.publish(Clock(clock=stamp(t)))
        odom = Odometry()
        odom.header.stamp = stamp(t)
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = frame['p']
        q = odom.pose.pose.orientation
        q.x, q.y, q.z, q.w = frame['q']
        # FlyCo's original control_odom carries world-frame linear velocities.
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = frame['v']
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = frame['w']
        self.odom.publish(odom)
        body_tf = TransformStamped()
        body_tf.header = odom.header
        body_tf.child_frame_id = 'base_link'
        body_tf.transform.translation.x, body_tf.transform.translation.y, body_tf.transform.translation.z = frame['p']
        body_tf.transform.rotation = q
        self.tf.sendTransform(body_tf)
        imu = Imu(header=odom.header)
        imu.header.frame_id = 'base_link'
        imu.orientation = q
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = frame['a']
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = frame['g']
        self.imu.publish(imu)
        gimbal = QuaternionStamped(header=odom.header)
        xyz = Rotation.from_euler('xyz', frame['gimbal'], degrees=True).as_quat()
        gimbal.quaternion.x, gimbal.quaternion.y, gimbal.quaternion.z, gimbal.quaternion.w = xyz.tolist()
        self.gimbal.publish(gimbal)
        self.telemetry['state_frames'] += 1
        if 'cloud' in arrays:
            self.cloud.publish(self.pointcloud(arrays['cloud'], t))
            self.telemetry['cloud_frames'] += 1
        if 'rgb' in arrays:
            rgb, depth = arrays['rgb'], arrays['depth']
            for values, encoding, publisher in [(rgb, 'rgb8', self.rgb), (depth, '32FC1', self.depth)]:
                msg = Image()
                msg.header.stamp = stamp(t)
                msg.header.frame_id = 'camera_optical'
                msg.height, msg.width = values.shape[:2]
                msg.encoding = encoding
                msg.step = values.strides[0]
                msg.data = array('B', values.tobytes())
                publisher.publish(msg)
            fx, fy, cx, cy = frame['intrinsics']
            info = CameraInfo()
            info.header = msg.header
            info.height, info.width = depth.shape
            info.distortion_model = 'plumb_bob'
            info.d = [0.]*5
            info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
            self.info.publish(info)
            position = np.asarray(frame['camera_position'])
            rotation = np.asarray(frame['camera_rotation'])
            points, colors = rgbd_points(depth, rgb, position, rotation, frame['intrinsics'])
            self.rgbcloud.publish(self.pointcloud(points, t, colors))
            tr = TransformStamped()
            tr.header = odom.header
            tr.child_frame_id = 'camera_optical'
            tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = position.tolist()
            x, y, z, w = Rotation.from_matrix(rotation).as_quat().tolist()
            tr.transform.rotation.x, tr.transform.rotation.y = x, y
            tr.transform.rotation.z, tr.transform.rotation.w = z, w
            self.tf.sendTransform(tr)
            self.telemetry['image_frames'] += 1
            self.telemetry['image_shape'] = list(rgb.shape)
            self.telemetry['rgbd_points'] = len(points)
        for action in frame.get('actions', []):
            self.handle_action(action)

    def serve(self):
        try:
            with socket.socket() as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind(('127.0.0.1', int(os.environ['FLYCO_ISAAC_PORT'])))
                server.listen(1)
                server.settimeout(1.)
                while not self.stop.is_set():
                    try:
                        conn, _ = server.accept()
                    except socket.timeout:
                        continue
                    with conn:
                        conn.settimeout(10.)
                        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        while not self.stop.is_set():
                            frame, arrays = recv_packet(conn)
                            self.publish(frame, arrays)
                            with self.lock:
                                send_packet(conn, {'control': self.control,
                                    'position_command': self.position_command,
                                    'gimbal': self.gimbal_command, 'intent': self.intent,
                                    'generation': self.generation, 'telemetry': self.telemetry,
                                    'ready_for_initial': self.ready_for_initial(),
                                    'ready_for_initialize': self.last_t >= 3. and
                                        self.odom.get_subscription_count() >= 3})
        except ConnectionError:
            self.stop.set()
        except Exception as exc:
            if not self.stop.is_set():
                (RUNTIME/'bridge_error.json').write_text(json.dumps({'error':repr(exc)}))
                self.get_logger().error(repr(exc))
                self.stop.set()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Bridge()
    # Let callbacks finish before invalidating their ROS entities.
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: node.stop.set())
    # Sensor I/O already has its own worker; these short callbacks can run
    # synchronously, leaving no queued executor callbacks during destruction.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.stop.is_set():
            executor.spin_once(timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        node.worker.join(timeout=1.)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
