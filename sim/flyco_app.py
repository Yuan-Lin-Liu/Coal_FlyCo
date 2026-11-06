"""Coal warehouse physics and RGB-D/LiDAR provider for FlyCo_ROS2."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'sim'), str(ROOT/'bridge')]
os.environ['PEGASUS_DISABLE_ROS2'] = '1'
parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=Path, default=ROOT/'coal_warehouse.usda')
parser.add_argument('--config', type=Path, default=ROOT/'config/coal_warehouse.yaml')
parser.add_argument('--runtime', type=Path, required=True)
parser.add_argument('--port', type=int, default=8870)
parser.add_argument('--headless', action='store_true')
parser.add_argument('--duration', type=float, default=0.)
args, kit_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + kit_args
from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless, 'hide_ui': args.headless,
    'width': 960, 'height': 540, 'window_width': 1280, 'window_height': 800,
    'multi_gpu': False, 'limit_cpu_threads': 8, 'renderer': 'RaytracedLighting',
    'anti_aliasing': 3, 'fast_shutdown': True})

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from pxr import UsdGeom, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.sensors.imu import IMU
from pegasus.simulator.params import ROBOTS
import omni.usd
from scene_storage import populate_stage
from flight_backend import FlyCoBackend
from batch_raycast import BatchRaycast
from wire import send_packet, recv_packet
from geometry import camera_pose


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2))
    temporary.replace(path)


result = {'ok': False, 'scene': str(args.scene.resolve())}
connection = None
try:
    cfg = yaml.safe_load(args.config.read_text())
    scene_hash = hashlib.sha256(args.scene.read_bytes()).hexdigest()
    populate_stage(omni.usd.get_context().get_stage(), args.scene.resolve())
    pg = PegasusInterface()
    pg._world = World(physics_dt=1/cfg['physics_hz'], rendering_dt=1/30,
                      stage_units_in_meters=1., backend='numpy', device='cpu')
    world = pg.world
    physics = world.get_physics_context()
    physics.enable_gpu_dynamics(False)
    physics.set_broadphase_type('MBP')
    physics.enable_fabric(False)
    physics.set_gravity(-9.81)
    use_px4 = cfg.get('flight_controller', 'flyco') in ('px4', 'px4-native')
    if use_px4:
        from px4_backend import WarehousePX4Backend
        backend = WarehousePX4Backend()
        backend.native_mode = cfg.get('flight_controller') == 'px4-native'
    else:
        backend = FlyCoBackend(cfg['full_thrust'])
    config = MultirotorConfig()
    config.backends = [backend]
    if use_px4:
        from pegasus.simulator.logic.sensors import Barometer, Magnetometer, GPS
        config.sensors = [IMU({'update_rate': cfg['physics_hz']}), Barometer(), Magnetometer(), GPS()]
    else:
        config.sensors = [IMU({'update_rate': cfg['physics_hz']})]
    render_every = max(1, round(cfg['physics_hz']/30))
    spawn_q = Rotation.from_euler('z', cfg.get('spawn_yaw_degrees', 0.), degrees=True).as_quat()
    drone = Multirotor('/World/Drone', ROBOTS['Iris'], 0, cfg['spawn'], spawn_q, config=config)
    camera = Camera('/World/FlyCoCamera', resolution=(cfg['width'], cfg['height']), frequency=30)
    camera.set_focal_length(.024)
    camera.set_horizontal_aperture(.024*cfg['width']/cfg['fx'])
    camera.set_clipping_range(.05, cfg['max_range'])
    world.reset()
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    initial_camera_position, initial_camera_rotation = camera_pose(
        cfg['spawn'], spawn_q, [0., 0., 0.], cfg['camera_offset'])
    initial_camera_q = Rotation.from_matrix(initial_camera_rotation).as_quat()
    camera.set_world_pose(position=initial_camera_position,
        orientation=initial_camera_q[[3, 0, 1, 2]], camera_axes='ros')
    for i in range(120):
        world.step(render=(i % render_every == 0))
    dc = drone.get_dc_interface()
    articulation = dc.get_articulation('/World/Drone')
    backend.mass = sum(float(dc.get_rigid_body_properties(dc.get_articulation_body(
        articulation, i)).mass) for i in range(dc.get_articulation_body_count(articulation)))
    backend.ground_height = float(drone.state.position[2])
    backend.sim_time = 0.
    raycast = BatchRaycast(world.stage)
    angles = [(az, el) for az in np.arange(cfg['lidar_azimuths'])*2*np.pi/cfg['lidar_azimuths']
              for el in np.linspace(np.deg2rad(-37), np.deg2rad(45), cfg['lidar_beams'])]
    directions = np.array([[np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)]
                           for az, el in angles])
    # Evaluation-only reference. It is never published as a predicted target.
    vertices = []
    cache = UsdGeom.XformCache()
    for prim in world.stage.Traverse():
        if 'Pile_' in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
            vertices.extend((np.c_[points, np.ones(len(points))] @
                             np.asarray(cache.GetLocalToWorldTransform(prim)))[:, :3].tolist())
    (args.runtime/'coal_reference.ply').write_text('ply\nformat ascii 1.0\nelement vertex '+
        str(len(vertices))+'\nproperty float x\nproperty float y\nproperty float z\nend_header\n'+
        ''.join(' '.join(map(str, p))+'\n' for p in vertices))
    write_json(args.runtime/'sim_metadata.json', {'mass_kg': backend.mass, 'physics_hz': cfg['physics_hz'],
        'scene_id': cfg.get('scene_id', 'custom'), 'scene_name': cfg.get('scene_name', args.scene.stem),
        'controller': ('FlyCo trajectory -> PX4 SITL -> Pegasus MAVLink rotor physics' if use_px4
                       else 'FlyCo ROS px4ctrl -> Isaac attitude adapter -> Pegasus rotor physics'),
        'state_source': 'Isaac simulation state (same role as FlyCo AirSim state)',
        'scene_sha256': scene_hash, 'colliders': len(raycast.paths), 'triangles': raycast.triangles,
        'camera': cfg, 'reference_use': 'RGB mapping evaluation only'})
    # Give the overview a predictable lens instead of reusing an editor zoom.
    overview_camera = UsdGeom.Camera(world.stage.GetPrimAtPath('/OmniverseKit_Persp'))
    if overview_camera:
        overview_camera.CreateFocalLengthAttr(24.)
        overview_camera.CreateHorizontalApertureAttr(36.)
        overview_camera.CreateVerticalApertureAttr(24.)
    set_camera_view(eye=np.array(cfg['overview_eye']), target=np.array(cfg['overview_target']))
    actions = []
    ui_notice = ['No action requested. Start with 1 Initialize when ready.']
    last_action_sequence = 0
    action_names = {'initialize': '1 Initialize', 'initial': '2 Initial flight',
                    'plan': '3 Enable planning', 'scan': '4 Autonomous scan',
                    'hold': 'Hold', 'land': 'Land', 'resume': 'Resume'}
    def request_action(name):
        actions.append(name)
        ui_notice[0] = 'Queued: '+action_names[name]+'; waiting for the ROS bridge.'
        feedback_label.text = ui_notice[0]
        print(f'[ACTION] queued {name} t={backend.sim_time:.2f}s', flush=True)
    follow_view = [False]
    def overview():
        follow_view[0] = False
        set_camera_view(eye=np.array(cfg['overview_eye']), target=np.array(cfg['overview_target']))
    if not args.headless:
        import omni.ui as ui
        window = ui.Window('FlyCo - Isaac Coal Warehouse', width=355, height=680)
        window.position_x, window.position_y = 900, 65
        buttons = {}
        with window.frame:
            with ui.VStack(spacing=7):
                ui.Label('FlyCo ROS2 + Isaac Sim | Pile '+str(cfg.get('scene_id', 'custom')), height=28)
                for name, label in [('initialize', '1 Initialize'), ('initial', '2 Initial flight'),
                                    ('plan', '3 Enable planning'), ('scan', '4 Autonomous scan')]:
                    buttons[name] = ui.Button(label, height=32, clicked_fn=lambda n=name: request_action(n))
                buttons['initialize'].enabled = False
                buttons['initial'].enabled = False
                with ui.HStack(height=32):
                    ui.Button('Hold', clicked_fn=lambda: request_action('hold'))
                    ui.Button('Land', clicked_fn=lambda: request_action('land'))
                    ui.Button('Resume', clicked_fn=lambda: request_action('resume'))
                with ui.HStack(height=30):
                    ui.Button('Overview', clicked_fn=overview)
                    ui.Button('Follow drone', clicked_fn=lambda: follow_view.__setitem__(0, True))
                ui.Label('Select the coal target in FlyCo-UI after initial flight.', word_wrap=True, height=45)
                status_label = ui.Label('', word_wrap=True, height=150)
                ui.Label('ACTION FEEDBACK (receipt is not completion)', height=22, style={'font_size':13})
                feedback_label = ui.Label(ui_notice[0], word_wrap=True, height=80, style={'font_size':14})
    deadline = time.monotonic()+90.
    while connection is None:
        try:
            connection = socket.create_connection(('127.0.0.1', args.port), timeout=1.)
        except OSError:
            if time.monotonic() > deadline:
                raise TimeoutError('FlyCo ROS bridge did not start')
            app.update()
            time.sleep(.1)
    connection.settimeout(10.)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    gimbal = [0., 0., 0.]
    generation = 0
    next_camera = next_lidar = next_status = 0.
    started = time.monotonic()
    step = 0
    image_count = 0
    previous_mode = None
    shutdown_requested = False
    shutdown_deadline = None
    trajectory = (args.runtime/'trajectory.jsonl').open('w')
    print('FLYCO_ISAAC_READY', flush=True)
    while app.is_running():
        wall = time.monotonic()
        if (args.runtime/'stop.request').exists() or (args.duration and backend.sim_time >= args.duration):
            if not use_px4:
                break
            if not shutdown_requested:
                if not backend.armed:
                    break
                backend.action('land')
                shutdown_requested = True
                shutdown_deadline = wall + 90.
                print('[FLIGHT] Shutdown requested; waiting for PX4 landing and stopped motors.', flush=True)
            if backend.landing_state == 'complete':
                break
            if wall > shutdown_deadline:
                raise TimeoutError('Shutdown landing did not complete within 90 seconds; inspect px4.log')
        world.step(render=False)
        step += 1
        t = backend.sim_time
        state = drone.state
        position = np.asarray(state.position)
        attitude = np.asarray(state.attitude)
        if not args.headless and follow_view[0] and step % render_every == 0:
            set_camera_view(eye=position+[-5., -5., 3.], target=position)
        arrays = {}
        frame = {'t': t, 'p': position.tolist(), 'q': attitude.tolist(),
                 'v': np.asarray(state.linear_velocity).tolist(),
                 'w': np.asarray(state.angular_velocity).tolist(), 'gimbal': gimbal,
                 'a': (np.asarray(backend.imu['linear_acceleration'])*[1, -1, -1]).tolist(),
                 'g': (np.asarray(backend.imu['angular_velocity'])*[1, -1, -1]).tolist(),
                 'actions': actions[:], 'intrinsics': [cfg[k] for k in ('fx', 'fy', 'cx', 'cy')]}
        actions.clear()
        if t >= next_lidar:
            body = Rotation.from_quat(attitude).as_matrix()
            origin = position+body@cfg['lidar_offset']
            rays = directions@body.T
            hits = raycast.raycast_batch(origin+rays*.45, rays, cfg['max_range'])
            arrays['cloud'] = hits[np.all(np.isfinite(hits), axis=1)].astype('<f4')
            next_lidar = t+1/cfg['lidar_hz']-.0001
        capture_camera = t >= next_camera
        if capture_camera or step % render_every == 0:
            # Keep the mount current on every render, including frames between
            # 10 Hz RGB-D publications. A stale mount can trail inside the
            # moving airframe and contaminate RTX temporal history.
            camera_position, optical_rotation = camera_pose(position, attitude, gimbal, cfg['camera_offset'])
            q = Rotation.from_matrix(optical_rotation).as_quat()
            camera.set_world_pose(camera_position, q[[3, 0, 1, 2]], camera_axes='ros')
            world.render()
        if capture_camera:
            rgb, depth = camera.get_rgb(), camera.get_depth()
            if rgb is not None and depth is not None and rgb.size and depth.size:
                arrays['rgb'] = np.ascontiguousarray(rgb[:, :, :3], dtype='u1')
                arrays['depth'] = np.ascontiguousarray(depth, dtype='<f4')
                frame.update(camera_position=camera_position.tolist(), camera_rotation=optical_rotation.tolist())
                image_count += 1
                if image_count % 20 == 0:
                    from PIL import Image
                    Image.fromarray(arrays['rgb']).save(args.runtime/'camera.tmp', format='PNG')
                    (args.runtime/'camera.tmp').replace(args.runtime/'camera.png')
                    np.save(args.runtime/'depth.npy', arrays['depth'])
                    write_json(args.runtime/'camera_pose.json', dict(t=t,
                        position=frame['camera_position'], rotation=frame['camera_rotation']))
            next_camera = t+1/cfg['camera_hz']-.0001
        send_packet(connection, frame, arrays)
        feedback, _ = recv_packet(connection)
        backend.set_command(feedback.get('position_command') if use_px4 else feedback['control'])
        gimbal = feedback['gimbal']
        if feedback['generation'] != generation:
            generation = feedback['generation']
            backend.action(feedback['intent'])
        action_feedback = feedback['telemetry'].get('last_action')
        if action_feedback and action_feedback['sequence'] != last_action_sequence:
            last_action_sequence = action_feedback['sequence']
            ui_notice[0] = (f'{action_feedback["state"].upper()} at '
                            f't={action_feedback["sim_time"]:.1f}s\n'+action_feedback['message'])
            print('[ACTION] '+ui_notice[0].replace('\n', ' '), flush=True)
        if backend.mode != previous_mode:
            previous_mode = backend.mode
            print(f'[FLIGHT] t={t:.2f}s mode={backend.mode} position={position.round(3).tolist()}'
                  f' fault={backend.fault or "none"}', flush=True)
        if t >= next_status:
            status = dict(t=t, position=frame['p'], velocity=frame['v'], mode=backend.mode,
                fault=backend.fault, motors=backend.input_ref, command_count=backend.command_count,
                image_frames=image_count, rtf=t/max(time.monotonic()-started, 1e-6),
                telemetry=feedback['telemetry'])
            if use_px4:
                status['px4'] = backend.telemetry()
            write_json(args.runtime/'sim_status.json', status)
            trajectory.write(json.dumps(status)+'\n')
            trajectory.flush()
            if not args.headless:
                buttons['initialize'].enabled = feedback['ready_for_initialize']
                buttons['initial'].enabled = feedback.get('ready_for_initial', False)
                telemetry = feedback['telemetry']
                coverage = telemetry.get('coverage')
                status_label.text = (f'{backend.mode}  t={t:.1f}s\n'+
                    f'Position: {position.round(2).tolist()}\n'+
                    f'FlyCo controls: {backend.command_count}\n'+backend.fault)
                status_label.text += (f'\nPredicted mesh: {telemetry.get("mesh_vertices", 0)} vertices\n'+
                    (f'Scan coverage: {100*coverage:.2f}%' if coverage is not None else 'Select a coal target after initial flight.'))
                feedback_label.text = ui_notice[0]
            next_status = t+.2
        time.sleep(max(0., 1/cfg['physics_hz']-(time.monotonic()-wall)))
    trajectory.close()
    # Landing may finish between the periodic snapshots. Persist the actual
    # stopped state so launch.py status cannot keep showing the previous arm.
    if use_px4 and 'status' in locals():
        status.update(t=backend.sim_time, position=np.asarray(backend.state.position).tolist(),
                      velocity=np.asarray(backend.state.linear_velocity).tolist(),
                      mode=backend.mode, fault=backend.fault,
                      motors=backend.input_ref, px4=backend.telemetry())
        write_json(args.runtime/'sim_status.json', status)
    result.update(ok=True, sim_seconds=backend.sim_time, control_count=backend.command_count,
                  image_frames=image_count, final_mode=backend.mode)
    if use_px4:
        result['px4'] = backend.telemetry()
except KeyboardInterrupt:
    result['stopped_by_operator'] = True
except Exception as exc:
    result.update(error=repr(exc), traceback=traceback.format_exc())
    traceback.print_exc()
finally:
    if connection:
        connection.close()
    result['scene_unchanged'] = hashlib.sha256(args.scene.read_bytes()).hexdigest() == locals().get('scene_hash')
    write_json(args.runtime/'result.json', result)
    if 'backend' in locals() and locals().get('use_px4'):
        backend.stop()
    app.close()
if result.get('error'):
    raise SystemExit(1)
