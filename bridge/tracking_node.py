"""Run FlyCo's SAM tracking algorithm with the Isaac camera pose convention."""
import sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

sys.path.insert(0, str(Path(__file__).resolve().parent))
from geometry import camera_pose
from scipy.spatial.transform import Rotation
from sam_tracking.ros.tracking_manager import SAMTrackingManager


class IsaacTracking(SAMTrackingManager):
    def _build_camera_pose_from_airsim(self, odom_msg, image_stamp=None):
        # Interface override only: segmentation, projection, clustering and
        # network weights are the original FlyCo_ROS2 implementation.
        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        angles = [0., 0., 0.]
        if self.gimbal_msg is not None:
            g = self.gimbal_msg.quaternion
            angles = Rotation.from_quat([g.x, g.y, g.z, g.w]).as_euler('xyz', degrees=True)
        position, rotation = camera_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], angles,
            self.get_parameter('isaac.camera_offset').value)
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rotation, position
        return transform


def main():
    rclpy.init()
    node = IsaacTracking('sam_tracking')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
