"""Bounded frame handoff and local model-process communication; no ROS imports."""
from dataclasses import dataclass
import json
import os
from pathlib import Path
import queue
import subprocess
import threading


@dataclass(frozen=True)
class FrameJob:
    frame: object
    text: str
    prompt_id: int


class FrameMailbox:
    """One pending frame, with prompt generations invalidating older results."""
    def __init__(self):
        self._condition = threading.Condition()
        self._latest = None
        self.text = ''
        self.prompt_id = 0
        self.closed = False

    def set_text(self, text):
        text = str(text).strip()
        with self._condition:
            if text == self.text:
                return False
            self.text = text
            self.prompt_id += 1
            self._latest = None
            self._condition.notify_all()
            return True

    def offer(self, frame):
        with self._condition:
            self._latest = frame
            self._condition.notify_all()

    def take(self, timeout=None):
        with self._condition:
            self._condition.wait_for(
                lambda: self.closed or (self.text and self._latest is not None), timeout)
            if self.closed or not self.text or self._latest is None:
                return None
            frame, self._latest = self._latest, None
            return FrameJob(frame, self.text, self.prompt_id)

    def is_current(self, job):
        with self._condition:
            return not self.closed and job.prompt_id == self.prompt_id

    def commit(self, job, publish):
        with self._condition:
            if not self.is_current(job):
                return False
            publish()
            return True

    def close(self):
        with self._condition:
            self.closed = True
            self._condition.notify_all()


def worker_environment():
    environment = dict(os.environ)
    for name in ('PYTHONPATH', 'PYTHONHOME', 'VIRTUAL_ENV', 'LD_LIBRARY_PATH',
                 'CUDA_HOME', 'CPATH', 'POINTNET2_OPS_EXTENSION_PATH',
                 'DINO_FFN_RKNN', 'SAM_LOW_MEMORY', 'SAM_DINO_SHORT_EDGE',
                 'SAM_DEFORMABLE_LIB', 'SAM_MEMENC_BACKEND'):
        environment.pop(name, None)
    environment.update(PYTHONNOUSERSITE='1', PYTHONUNBUFFERED='1',
                       SAM_ENCODER_BACKEND='torch', SAM_MEMORY_BACKEND='torch',
                       PYTHONUTF8='1')
    return environment


class WorkerStopped(RuntimeError):
    """The child exited, timed out, or violated the local pipe protocol."""


class WorkerClient:
    def __init__(self, python, source_dir):
        script = Path(__file__).with_name('semantic_worker.py')
        self.process = subprocess.Popen(
            [str(python), '-u', '-B', str(script), '--source-dir', str(source_dir)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None,
            env=worker_environment(), start_new_session=True)
        self._replies = queue.Queue()
        self._reader = threading.Thread(target=self._read, name='semantic-replies', daemon=True)
        self._reader.start()

    def _read(self):
        try:
            for line in self.process.stdout:
                self._replies.put(json.loads(line))
        except (ValueError, OSError) as error:
            self._replies.put({'kind': 'fatal', 'error': 'Worker protocol error: ' + str(error)})
        finally:
            self._replies.put({'kind': 'fatal', 'error': 'Model worker exited; inspect its terminal log'})

    def _receive(self, timeout):
        try:
            result = self._replies.get(timeout=timeout)
        except queue.Empty as error:
            self.close()
            raise WorkerStopped('Model worker timed out after %.1fs' % timeout) from error
        if result.get('kind') == 'fatal':
            raise WorkerStopped(result.get('error', 'Model worker failed'))
        if result.get('kind') == 'error':
            raise RuntimeError(result.get('error', 'Inference failed'))
        return result

    def wait_ready(self, timeout):
        result = self._receive(timeout)
        if result.get('kind') != 'ready':
            raise WorkerStopped('Model worker did not advertise readiness')
        return result

    def segment(self, request, timeout):
        try:
            self.process.stdin.write((json.dumps(request, ensure_ascii=False) + '\n').encode('utf-8'))
            self.process.stdin.flush()
        except (BrokenPipeError, OSError) as error:
            raise WorkerStopped('Model worker pipe closed') from error
        result = self._receive(timeout)
        if (result.get('kind') != 'result' or result.get('id') != request['id']
                or result.get('prompt_id') != request['prompt_id']
                or result.get('header') != request['header']):
            raise WorkerStopped('Model result does not match the submitted image/prompt')
        return result

    def close(self):
        if self.process.poll() is None:
            try:
                self.process.terminate()
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self._reader.join(timeout=1)
        for stream in (self.process.stdin, self.process.stdout):
            if stream is not None:
                stream.close()
