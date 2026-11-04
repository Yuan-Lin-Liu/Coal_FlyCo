#!/usr/bin/env python3
"""Project-local Semantic Perception service, isolated from ROS2's Python/Torch and other FlyCo sessions."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'third_party/SAM_AD_Perception'
NAME='coal-flyco-sam-a'
ENDPOINT='http://127.0.0.1:8851'
IMAGE=os.environ.get('COAL_SAM_A_IMAGE','coal-flyco/sam-a:20261103')


def fingerprint():
    digest=hashlib.sha256()
    for folder in ('sam_ad','runtime_api','runtime_app'):
        for path in sorted((SOURCE/folder).rglob('*.py')):
            digest.update(path.relative_to(SOURCE).as_posix().encode());digest.update(path.read_bytes())
    return digest.hexdigest()


def health(endpoint=ENDPOINT):
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(endpoint.rstrip('/')+'/health',timeout=5)as response:
        return json.load(response)


def validate(info):
    if info.get('protocol')!='flyco-text-sam-v1' or info.get('backend')!='A':
        raise RuntimeError('Expected an semantic perception service at '+ENDPOINT)
    if info.get('source_fingerprint')!=fingerprint():
        raise RuntimeError('Semantic perception service code differs from this project; stop it with tools/semantic_perception_service.py stop, then restart.')
    if info.get('status')!='ready':raise RuntimeError('Semantic perception service is not ready: '+str(info.get('status')))
    return info


def running():
    r=subprocess.run(['docker','inspect','-f','{{.State.Running}}',NAME],capture_output=True,text=True)
    return r.returncode==0 and r.stdout.strip()=='true'


def start():
    started=not running()
    if started:
        # Remove a previous failed container before reusing its stable name.
        subprocess.run(['docker','rm','-f',NAME],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        subprocess.run(['python3',str(SOURCE/'prepare_assets.py'),'--backend','A','--check'],check=True,stdout=subprocess.DEVNULL)
        subprocess.run(['docker','image','inspect',IMAGE],check=True,stdout=subprocess.DEVNULL)
        logs=ROOT/'runtime/sam-a-traces';logs.mkdir(parents=True,exist_ok=True)
        state=ROOT/'runtime/sam-a-service';state.mkdir(parents=True,exist_ok=True)
        (SOURCE/'runtime').mkdir(exist_ok=True)
        subprocess.run(['docker','run','-d','--init','--name',NAME,
            '--label','org.coal-flyco.role=sam-a','--network','host','--gpus','all',
            '--user',str(os.getuid())+':'+str(os.getgid()),'--cpus','6','--memory','12g',
            '-e','HOME=/tmp','-e','SAM_AD_A_MODEL=large',
            '-e','SAM_AD_TRACE_DIR=/trace',
            '-e','NVIDIA_DRIVER_CAPABILITIES=compute,utility',
            '-v',str(SOURCE)+':/app:ro','-v',str(logs)+':/trace:rw',
            '-v',str(state)+':/app/runtime:rw',
            '-w','/app',IMAGE,'python','-B','-m','sam_ad.server',
            '--backend','A','--bind','127.0.0.1','--port','8851'],check=True,stdout=subprocess.DEVNULL)
    deadline=time.monotonic()+180;last='starting'
    try:
        while time.monotonic()<deadline:
            try:
                info=health()
                if info.get('status')=='ready':return dict(started=started,health=validate(info))
                if info.get('status')=='error':raise RuntimeError('Semantic perception model load failed: '+str(info))
                last=info.get('status')
            except OSError as exc:last=str(exc)
            if not running():raise RuntimeError('Semantic perception container exited; inspect docker logs '+NAME)
            time.sleep(1)
        raise RuntimeError('Timed out waiting for Semantic perception service: '+str(last))
    except BaseException:
        if started:stop()
        raise


def stop():
    subprocess.run(['docker','stop','-t','20',NAME],check=False,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.run(['docker','rm','-f',NAME],check=False,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('start','stop','status'))
    args=parser.parse_args()
    if args.action=='start':print(json.dumps(start(),ensure_ascii=False,indent=2))
    elif args.action=='stop':stop()
    else:print(json.dumps(validate(health()),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
