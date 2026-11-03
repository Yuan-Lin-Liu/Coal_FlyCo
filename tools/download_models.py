#!/usr/bin/env python3
"""Download verified model files, or prepare fixed Hugging Face snapshots."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=ROOT/'.local/flyco/models')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--huggingface', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((ROOT/'config/models.json').read_text())
    target = args.directory.resolve()
    missing = []
    if not args.huggingface:
        for entry in manifest['files']:
            path = target/entry['path']
            if path.is_file() and digest(path) == entry['sha256']:
                print('Verified:', entry['path'], flush=True)
                continue
            if args.check:
                missing.append(entry['path'])
                continue
            path.parent.mkdir(parents=True, exist_ok=True)
            partial = path.with_name(path.name + '.part')
            url = os.environ.get('MESH_CHECKPOINT_URL', entry['url']) if path.name == 'checkpoint.pt' else entry['url']
            subprocess.run(['curl', '--fail', '--location', '--retry', '5',
                            '--continue-at', '-', url, '--output', str(partial)], check=True)
            if digest(partial) != entry['sha256']:
                raise RuntimeError('SHA256 mismatch (or download quota/HTML response): '+str(partial))
            partial.replace(path)
    if args.huggingface or args.check:
        hub = target/'huggingface/hub'
        for entry in manifest['huggingface']:
            repository = hub/('models--'+entry['repo'].replace('/', '--'))
            snapshot = repository/'snapshots'/entry['revision']
            reference = repository/'refs/main'
            if args.check:
                files = list(snapshot.rglob('*')) if snapshot.exists() else []
                if not (snapshot/'config.json').is_file() or not any(
                        p.is_file() and p.suffix in ('.bin', '.safetensors') for p in files):
                    missing.append(entry['repo']+'@'+entry['revision'])
                if not reference.is_file() or reference.read_text().strip() != entry['revision']:
                    missing.append(entry['repo']+' cache reference')
            else:
                from huggingface_hub import snapshot_download
                snapshot_download(entry['repo'], revision=entry['revision'], cache_dir=hub)
                reference.parent.mkdir(parents=True, exist_ok=True)
                reference.write_text(entry['revision'])
                print('Prepared:', entry['repo'], entry['revision'], flush=True)
    if missing:
        raise RuntimeError('Missing or invalid resources: '+', '.join(missing))


if __name__ == '__main__':
    main()
