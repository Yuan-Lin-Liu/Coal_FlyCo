"""Exercise the real ROS callback when recovery has no image payload."""
from types import SimpleNamespace
import unittest
import numpy as np
try:
    from sam_tracking.ros.tracking_manager import SAMTrackingManager
except ImportError:
    SAMTrackingManager = None


@unittest.skipIf(SAMTrackingManager is None, 'Run inside the project ROS2 environment')
class RecoveryCallbackTests(unittest.TestCase):
    def run_state(self, result):
        missing = []
        stamp = SimpleNamespace(sec=12, nanosec=34)
        header = SimpleNamespace(stamp=stamp)
        image = SimpleNamespace(header=header)
        pose = SimpleNamespace(position=None, orientation=None)
        node = SimpleNamespace(
            img_odom_data=[{'img': image, 'odom': SimpleNamespace(pose=SimpleNamespace(pose=pose))}],
            camera_info=object(), pointcloud_msgs=[object()], airsim_camera_pose_enabled=False,
            _pose_msg_to_matrix=lambda *args: np.eye(4),
            _bridge=SimpleNamespace(imgmsg_to_cv2=lambda *args, **kw: np.zeros((480,640,3),np.uint8)),
            provider=SimpleNamespace(initialized=True, begin_frame=lambda token: None),
            pipeline=SimpleNamespace(sync_provider_state=lambda **kw: (True,None,None,None),
                                     track_frame=lambda **kw: result),
            mask_T=SimpleNamespace(mask_with_transform=None),
            depth_guidance=SimpleNamespace(enabled=False), _load_depth_mask=lambda stamp: None,
            _publish_unavailable_mask=lambda rgb, h: missing.append(h), frame_idx=0)
        SAMTrackingManager.tracking_timer_callback(node)
        self.assertEqual(missing, [header])
        self.assertEqual(node.frame_idx, 0)

    def test_pending_recovery_without_mask_does_not_crash_the_callback(self):
        self.run_state({'state':'reinit_pending', 'track_mask':None,
                        'reinit_reason':'depth_empty_mask_reinit'})

    def test_no_prediction_clears_the_display_without_publishing_geometry(self):
        self.run_state(None)

if __name__ == '__main__':unittest.main()
