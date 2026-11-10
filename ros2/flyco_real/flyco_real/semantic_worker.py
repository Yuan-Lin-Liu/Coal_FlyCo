"""Python 3.12 model owner, invoked by the ROS process over local pipes."""
import argparse
import base64
import json
import os
from pathlib import Path
import sys
import time
import traceback


class LocalSegmenter:
    def __init__(self, source_dir):
        source = Path(source_dir).expanduser().absolute()
        sys.path.insert(0, str(source))
        # Reuse the existing backend directly; no HTTP server or manifest verification.
        from runtime_api.models import build_models
        self.make_pipe, self.parse, _, _, metadata = build_models(
            'A', source, source / 'runtime_app')
        self.metadata = {key: metadata.get(key) for key in
                         ('backend', 'device', 'precision', 'segmentation_model')}
        self.pipe = None
        self.prompt_id = None
        self.text = None
        self.index = 0

    def segment(self, request):
        import cv2
        import numpy as np
        import torch
        bgr = cv2.imdecode(np.frombuffer(
            base64.b64decode(request['image_png'], validate=True), np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError('Cannot decode input image')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        command = None
        if (self.pipe is None or self.prompt_id != request['prompt_id']
                or self.text != request['text']):
            command = self.parse(request['text'])
            if command.quantity != 'one':
                raise ValueError('This FlyCo segmentation entrypoint supports one target')
            self.pipe = self.make_pipe()
            self.prompt_id, self.text, self.index = request['prompt_id'], request['text'], 0
        tick = time.monotonic()
        with torch.inference_mode():
            result = self.pipe.process(rgb, self.index, 1_000_000_000, command)
        self.index += 1
        masks = result.get('masks', {})
        if len(masks) > 1:
            raise ValueError('Model returned more than one target mask')
        mask = next(iter(masks.values()), None)
        encoded = None
        if mask is not None:
            binary = (np.asarray(mask) > 0).astype(np.uint8) * 255
            if binary.shape != bgr.shape[:2]:
                binary = cv2.resize(binary, (bgr.shape[1], bgr.shape[0]),
                                    interpolation=cv2.INTER_NEAREST)
            ok, buffer = cv2.imencode('.png', binary)
            if not ok:
                raise RuntimeError('Cannot encode model mask')
            encoded = base64.b64encode(buffer).decode('ascii')
        return {'mask_png': encoded, 'phase': result.get('phase', ''),
                'inference_ms': (time.monotonic() - tick) * 1000,
                'target_ids': [int(value) for value in masks]}

    def reset(self):
        self.pipe = None


def run(source_dir, inputs, outputs):
    def emit(message):
        outputs.write((json.dumps(message, ensure_ascii=False) + '\n').encode('utf-8'))
        outputs.flush()

    # All model/library prints go to stderr; stdout belongs exclusively to IPC.
    sys.stdout = sys.stderr
    try:
        model = LocalSegmenter(source_dir)
        emit({'kind': 'ready', 'model': model.metadata})
    except Exception as error:
        traceback.print_exc()
        emit({'kind': 'fatal', 'error': 'Model loading failed: ' + str(error)})
        return 1
    for line in inputs:
        request = {}
        try:
            request = json.loads(line)
            if request.get('kind') == 'quit':
                return 0
            if request.get('kind') != 'frame':
                raise ValueError('Expected an image frame')
            result = model.segment(request)
            emit({'kind': 'result', 'id': request['id'], 'prompt_id': request['prompt_id'],
                  'header': request['header'], **result})
        except Exception as error:
            model.reset()
            traceback.print_exc()
            emit({'kind': 'error', 'id': request.get('id'), 'error': str(error)})
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', required=True)
    args = parser.parse_args()
    os.environ.update(SAM_ENCODER_BACKEND='torch', SAM_MEMORY_BACKEND='torch')
    # Duplicate the protocol pipe before redirecting fd 1, so native libraries'
    # printf/write calls cannot corrupt messages either.
    with os.fdopen(os.dup(sys.stdout.fileno()), 'wb') as output:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        return run(args.source_dir, sys.stdin.buffer, output)


if __name__ == '__main__':
    raise SystemExit(main())
