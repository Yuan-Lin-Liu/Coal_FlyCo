"""Read saved process logs incrementally without changing the running session."""
import codecs
import json
from pathlib import Path
import re
import time


IMPORTANT = (
    '[ACTION]', 'FLYCO_ISAAC_READY', 'Simulation App Startup Complete',
    'Operator action:', '[Keys]', 'LB pressed', '[2 / Y]', 'Initial ->',
    'Successfully select', '[3 / X]', 'RB pressed', '[4 / RB]',
    'SAMTrackingManager has been initialized', 'MeshPredictNode Init Done',
    'Flight Coverage', '[ERROR]', '[Error]', 'Traceback', 'Error:',
    ' Killed ', 'rejected', 'Upstream control stale',
)
ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
# Timestamps change on repeated messages; keep the logger, severity and body.
ROS_TIME = re.compile(r'\[\d+(?:\.\d+)?\](?=\s+\[[^\]]+\]:)')
KIT_TIME = re.compile(r'^\d{4}-\d{2}-\d{2}T\S+\s+\[[^\]]+\]\s*')
ACTION_EVENTS = ('[ACTION]', 'Operator action:', 'LB pressed', '[2 / Y]',
                 'Initial ->', 'Successfully select', '[3 / X]', 'RB pressed', '[4 / RB]')


class ConsoleOutput:
    def __init__(self, run, verbose=False):
        self.run = Path(run)
        self.verbose = verbose
        self.offsets = {}
        self.buffers = {}
        self.decoders = {}
        self.next_status = 0.
        self.last_mode = None
        self.seen_messages = set()

    def should_show(self, filename, line):
        if self.verbose:
            return True
        if not any(key in line for key in IMPORTANT):
            return False
        # A repeated button press is a new event and still needs a receipt.
        if any(key in line for key in ACTION_EVENTS):
            return True
        message = ROS_TIME.sub('', KIT_TIME.sub('', line))
        # Coverage values keep updating in the panel; announce this log once.
        if 'Flight Coverage' in message:
            message = message.split('Flight Coverage', 1)[0]+'Flight Coverage'
        key = (filename, message)
        if key in self.seen_messages:
            return False
        self.seen_messages.add(key)
        return True

    def poll(self):
        output = []
        for filename, label in [('isaac.log', 'Isaac'), ('ros.log', 'ROS')]:
            path = self.run/filename
            try:
                with path.open('rb') as stream:
                    offset = self.offsets.get(filename, 0)
                    if path.stat().st_size < offset:
                        offset = 0
                        self.buffers.pop(filename, None)
                        self.decoders.pop(filename, None)
                    stream.seek(offset)
                    data = stream.read()
                    self.offsets[filename] = stream.tell()
            except FileNotFoundError:
                continue
            decoder = self.decoders.setdefault(filename,
                codecs.getincrementaldecoder('utf-8')(errors='replace'))
            text = self.buffers.get(filename, '')+decoder.decode(data)
            lines = text.split('\n')
            self.buffers[filename] = lines.pop()
            for line in lines:
                line = ANSI.sub('', line).replace('\x00', '').rstrip()
                if line and self.should_show(filename, line):
                    output.append(f'[{label}] {line}')
        try:
            status = json.loads((self.run/'sim_status.json').read_text())
        except (OSError, ValueError):
            return output
        mode = (status.get('mode'), status.get('fault'))
        if mode != self.last_mode or (self.verbose and time.monotonic() >= self.next_status):
            self.next_status = time.monotonic()+5.
            self.last_mode = mode
            telemetry = status.get('telemetry', {})
            position = ', '.join(f'{x:.2f}' for x in status.get('position', []))
            coverage = telemetry.get('coverage')
            progress = f'{coverage*100:.2f}%' if coverage is not None else '等待扫描'
            output.append(f'[状态] t={status.get("t", 0):.1f}s  {status.get("mode")}  '
                f'位置=({position})  控制消息={status.get("command_count", 0)}  '
                f'网格={telemetry.get("mesh_vertices", 0)}顶点  覆盖={progress}')
            if status.get('fault'):
                output.append('[故障] '+status['fault'])
        return output

    def emit(self):
        for line in self.poll():
            print(line, flush=True)
