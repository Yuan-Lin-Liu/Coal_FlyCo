#!/usr/bin/env python3
"""SITL-only acceptance of the registered FlyCo mode in an active native session."""
import json,time,uuid
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool,Trigger
from px4_msgs.msg import VehicleCommand
from flyco_real.px4_dds import topic
rclpy.init();n=rclpy.create_node('flyco_native_sim_verifier');control={};stages=[];run=(Path(__file__).resolve().parents[1]/'runtime/latest').resolve()
session=json.loads((run/'session.json').read_text())
if session.get('flight_controller') != 'px4-native' or not session.get('px4_dir') or session.get('ended'):
 raise SystemExit('An active px4-native SITL session is required')
n.create_subscription(String,'/flyco/control/status',lambda m:control.update(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub=n.create_publisher(VehicleCommand,topic('/fmu','in','vehicle_command',VehicleCommand),10)
def save(): (run/'native-mode-verification.json').write_text(json.dumps({'stages':stages,'control':control},indent=2))
def wait(label,check,seconds=30):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.05)
  if check():stages.append(label);save();print(label,'PASS',flush=True);return
 raise RuntimeError(label+' timeout: '+str(control))
def command(cmd,**params):
 m=VehicleCommand(timestamp=int(json.loads((run/'sim_status.json').read_text())['t']*1e6),command=cmd,target_system=1,target_component=1,source_system=245,source_component=192,from_external=True,**params);pub.publish(m)
def service(name,kind=Trigger,data=None):
 c=n.create_client(kind,'/flyco/control/'+name);assert c.wait_for_service(timeout_sec=3)
 f=c.call_async(kind.Request() if data is None else kind.Request(data=data))
 end=time.monotonic()+5
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 result=f.result();n.destroy_client(c)
 if not result or not result.success:raise RuntimeError(name+': '+str(result))
def action(name,**kw):(run/'actions'/f'{time.time_ns()}-{uuid.uuid4().hex}.json').write_text(json.dumps(dict(action=name,**kw)))
def status():return json.loads((run/'sim_status.json').read_text())
try:
 wait('registered',lambda:control.get('registered_mode_id') is not None)
 action('initialize')
 wait('planner_initialized',lambda:json.loads((run/'bridge_status.json').read_text()).get('ready_for_initial'))
 wait('px4_ready',lambda:control.get('px4_ok'))

 if not control.get('enabled'):service('enable',SetBool,True)
 command(VehicleCommand.VEHICLE_CMD_SET_NAV_STATE,param1=float(control['registered_mode_id']))
 wait('FlyCo_selected',lambda:control.get('mode')=='FlyCo')
 executor=n.create_publisher(VehicleCommand,topic('/fmu','in','vehicle_command_mode_executor',VehicleCommand),10)
 end=time.monotonic()+1.
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 executor.publish(VehicleCommand(command=400,param1=1.,source_component=VehicleCommand.COMPONENT_MODE_EXECUTOR_START+int(control['mode_executor_id'])))
 wait('normally_armed',lambda:control.get('armed'))
 end=time.monotonic()+1.5
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 service('takeoff')
 wait('FlyCo_takeoff',lambda:abs(status()['position'][2]-2.)<.15,90)
 wait('returned_to_FlyCo',lambda:control.get('takeoff_phase')=='complete' and control.get('mode')=='FlyCo',45)
 action('text',text='A single large pile of black coal.')
 wait('predicted_mesh',lambda:status().get('telemetry',{}).get('mesh_vertices',0)>0,120)
 action('plan');end=time.monotonic()+1
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 action('scan');origin=status()['position'];count=status()['command_count']
 wait('FlyCo_scan',lambda:control.get('tracking') and abs(status()['position'][0]-origin[0])>.6,120)
 wait('FlyCo_scan_complete',lambda:status().get('telemetry',{}).get('flight_end'),300)
 service('land')
 wait('land_mode',lambda:control.get('mode')=='AUTO.LAND')
 wait('disarmed',lambda:not control.get('armed') and
      max(map(abs,status()['motors']))<1. and status()['position'][2]<.3,90)
 stages.append('SUCCESS');save()
except Exception as e:
 print(str(e),flush=True);stages.append('ERROR: '+str(e));save()
finally:
 if control.get('armed'):
  try:
   service('land')
   wait('cleanup_disarmed',lambda:not control.get('armed'),90)
  except Exception as cleanup_error:
   stages.append('Cleanup requires session shutdown: '+str(cleanup_error));save()
 (run/'stop.request').touch()
 n.destroy_node();rclpy.shutdown()
if not stages or stages[-1] != 'SUCCESS':
 raise SystemExit(1)
