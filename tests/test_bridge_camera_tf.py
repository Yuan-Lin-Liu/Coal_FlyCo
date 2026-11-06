"""Camera world poses must not acquire the IMU's body-relative header."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


class Recorder:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(copy.deepcopy(msg))
    def sendTransform(self, msg): self.publish(msg)


@pytest.mark.parametrize('position,yaw', [([-14.,0.,2.],0.), ([5.,-2.,3.],np.pi/2)])
def test_camera_tf_uses_world_parent_and_single_mount_offset(tmp_path, monkeypatch, position, yaw):
    monkeypatch.setenv('FLYCO_ISAAC_RUNTIME', str(tmp_path))
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('camera_tf_bridge', root/'bridge/ros_bridge.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from geometry import camera_pose
    q = Rotation.from_euler('z', yaw).as_quat()
    camera, rotation = camera_pose(position, q, [0.,0.,0.])
    recorders = {name: Recorder() for name in ('clock','odom','tf','imu','gimbal','rgb','depth','info','rgbcloud')}
    adapter = SimpleNamespace(**recorders, last_t=0., telemetry=dict(state_frames=0, image_frames=0),
                              pointcloud=module.Bridge.pointcloud)
    frame = dict(t=1., p=position, q=q.tolist(), v=[0.,0.,0.], w=[0.,0.,0.],
        a=[0.,0.,9.81], g=[0.,0.,0.], gimbal=[0.,0.,0.], intrinsics=[2.,2.,1.,1.],
        camera_position=camera.tolist(), camera_rotation=rotation.tolist())
    module.Bridge.publish(adapter, frame, dict(rgb=np.zeros((2,2,3),dtype=np.uint8),
                                              depth=np.ones((2,2),dtype=np.float32)))
    transforms = {m.child_frame_id: m for m in recorders['tf'].messages}
    assert transforms['base_link'].header.frame_id == 'world'
    assert transforms['camera_optical'].header.frame_id == 'world'
    assert recorders['odom'].messages[0].header.frame_id == 'world'
    assert recorders['imu'].messages[0].header.frame_id == 'base_link'
    assert recorders['gimbal'].messages[0].header.frame_id == 'world'
    point = transforms['camera_optical'].transform.translation
    camera_world = np.array([point.x,point.y,point.z])
    np.testing.assert_allclose(camera_world-np.asarray(position),
                               Rotation.from_quat(q).apply([.125,0.,0.]), atol=1.e-9)
    # An optical point directly ahead projects once from the measured camera origin.
    forward = camera_world+rotation @ [0.,0.,3.]
    np.testing.assert_allclose(forward-np.asarray(position),
                               Rotation.from_quat(q).apply([3.125,0.,0.]), atol=1.e-9)
