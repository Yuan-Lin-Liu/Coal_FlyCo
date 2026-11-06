"""Read-only semantic service readiness check, available through ros2 run."""
import argparse
import http.client
import json
import math
import os
from pathlib import Path
import time
from urllib.parse import urlsplit
import yaml
from sam_tracking.providers.http_text_sam_provider import PROTOCOL, REVISION


def fetch_health(endpoint, token, timeout=3.):
    url = urlsplit(endpoint)
    if url.scheme not in ('http', 'https') or not url.hostname or url.username:
        raise ValueError('Expected an HTTP(S) segmentation_endpoint without embedded credentials')
    cls = http.client.HTTPSConnection if url.scheme == 'https' else http.client.HTTPConnection
    connection = cls(url.hostname, url.port, timeout=timeout)
    try:
        headers = {'Authorization': 'Bearer ' + token} if token else {}
        connection.request('GET', url.path.rstrip('/') + '/health', headers=headers)
        response = connection.getresponse()
        if response.status != 200:
            raise RuntimeError('Semantic health HTTP %d; check endpoint and SAM_AD_API_TOKEN' % response.status)
        data = json.loads(response.read())
    finally:
        connection.close()
    if (data.get('protocol') != PROTOCOL or data.get('service_revision') != REVISION or
            data.get('backend') != 'A'):
        raise ValueError('Semantic backend or protocol version does not match this FlyCo client')
    if data.get('status') not in ('ready', 'loading'):
        raise RuntimeError('Semantic service failed: ' + str(data.get('error', data.get('status'))))
    return data['status']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--wait', action='store_true', help='Wait for startup/readiness')
    parser.add_argument('--timeout', type=float, default=120., help='Overall wait limit in seconds')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0.:
        parser.error('--timeout must be positive and finite')
    try:
        settings = yaml.safe_load(args.config.expanduser().read_text())
        endpoint = settings['sam_tracking']['ros__parameters']['segmentation_endpoint']
        deadline = time.monotonic() + args.timeout
        last_status = None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.: raise TimeoutError('Timed out waiting for semantic readiness')
            try:
                status = fetch_health(endpoint, os.environ.get('SAM_AD_API_TOKEN', ''), min(3., remaining))
            except (OSError, http.client.HTTPException):
                if not args.wait: raise
                status = 'unreachable'
            if status != last_status:
                print('Semantic service: ' + status, flush=True)
                last_status = status
            if status == 'ready': return 0
            if not args.wait: return 1
            time.sleep(min(1., max(0., deadline - time.monotonic())))
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError, KeyError, http.client.HTTPException) as error:
        print('Semantic check failed: ' + str(error), flush=True)
        return 1
