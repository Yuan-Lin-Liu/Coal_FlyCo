"""Display the measured camera frustum, independently of planned trajectories."""
import math

from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException


def frustum_points(info, distance):
    """Eight frustum edges in optical coordinates: X right, Y down, Z forward."""
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if (info.width <= 0 or info.height <= 0 or distance <= 0. or
            not all(math.isfinite(v) for v in (fx, fy, cx, cy, distance)) or fx <= 0. or fy <= 0.):
        raise ValueError('Need calibrated image dimensions, focal lengths and a positive distance')
    corners = [((u-cx)*distance/fx, (v-cy)*distance/fy, distance)
               for u,v in ((0.,0.),(float(info.width),0.),
                           (float(info.width),float(info.height)),(0.,float(info.height)))]
    edges = [((0.,0.,0.),p) for p in corners]
    edges += [(corners[i],corners[(i+1)%4]) for i in range(4)]
    return [Point(x=x,y=y,z=z) for edge in edges for x,y,z in edge]


class CameraFOV(Node):
    def __init__(self):
        super().__init__('camera_fov', automatically_declare_parameters_from_overrides=True)
        def param(name, default):
            if not self.has_parameter(name): self.declare_parameter(name, default)
            return self.get_parameter(name).value
        simulation = param('simulation', False)
        info_topic = param('camera_info_topic', '/isaac/camera/info' if simulation else '/flyco/camera/info')
        self.world = param('world_frame', 'world')
        self.distance = float(param('distance', 3.))
        self.max_age = float(param('pose_timeout', .5))
        if not math.isfinite(self.distance) or self.distance <= 0.:
            raise ValueError('distance must be finite and positive')
        self.output = self.create_publisher(Marker, param('output_topic', '/flyco/camera/fov'), 1)
        self.info = self.points = None
        self.visible = False
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE if simulation else DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos)
        self.create_timer(.1, self.tick)
        self.get_logger().info('Camera FOV waits for calibration and current optical TF: '+info_topic)

    def on_info(self, msg):
        if not msg.header.frame_id:
            return
        try:
            self.points = frustum_points(msg, self.distance)
            self.info = msg
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)

    def marker(self):
        msg = Marker(ns='measured_camera_FOV', id=0, type=Marker.LINE_LIST)
        msg.pose.orientation.w = 1.
        return msg

    def tick(self):
        if self.info is None:
            return
        msg = self.marker()
        msg.header.frame_id = self.info.header.frame_id
        try:
            transform = self.buffer.lookup_transform(self.world, self.info.header.frame_id, Time())
            stamp = transform.header.stamp
            age = self.get_clock().now().nanoseconds*1.e-9-(stamp.sec+stamp.nanosec*1.e-9)
            if age < -.1 or age > self.max_age:
                raise ValueError('Camera pose is stale')
        except (TransformException, ValueError):
            if self.visible:
                msg.action = Marker.DELETE
                self.output.publish(msg)
                self.visible = False
            return
        msg.action = Marker.ADD
        msg.frame_locked = True
        msg.points = self.points
        msg.color.r, msg.color.a, msg.scale.x = 1., 1., .06
        msg.lifetime.nanosec = 500000000
        self.output.publish(msg)
        self.visible = True


def main():
    from flyco_common.execution import main_for
    main_for(CameraFOV)
