#!/usr/bin/env python3
"""Independent simulation-only launcher. Uses installed environment verbatim."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
PID=ROOT/'.sim-demo-process.json'
PYTHON=ROOT/'mujoco_demo/.venv-cpu/bin/python'


def identity(pid):
    try:
        root=Path('/proc')/str(pid);stat=root.joinpath('stat').read_text().rsplit(')',1)[1].split()
        if stat[0]=='Z':return None
        return {'pid':pid,'start_ticks':stat[19],'command':root.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode()}
    except FileNotFoundError:return None


def owned():
    if not PID.exists():return None
    saved=json.loads(PID.read_text());actual=identity(saved['identity']['pid'])
    if actual is None:return None
    if actual!=saved['identity'] or 'sim_demo_main.py' not in actual['command']:raise SystemExit('Process identity mismatch')
    return saved


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['start','stop','status'])
    p.add_argument('--output',type=Path)
    p.add_argument('--resolution',choices=['960x540','640x360'],default='960x540')
    p.add_argument('--host',default='127.0.0.1',help='HTTP/WebSocket bind address; use 0.0.0.0 for LAN access')
    p.add_argument('--port',type=int,default=8091,help='HTTP/WebSocket port')
    p.add_argument('--chassis-config',type=Path,default=ROOT/'custom-chassis-physics-20261012/assets/dm-h65-casters-20261012/config.json')
    p.add_argument('--task-config',type=Path,default=ROOT/'dm-h65-continuous-20261013/simple-task.json')
    args=p.parse_args();current=owned()
    if args.action=='status':
        if not current:print(json.dumps({'running':False}));return
        host=current.get('host','127.0.0.1')
        health_host='127.0.0.1' if host in {'0.0.0.0','::'} else host
        with urllib.request.urlopen(f'http://{health_host}:{current.get("port",8091)}/api/health',timeout=5) as response:health=json.load(response)
        print(json.dumps({'running':True,'process':current,'health':health},ensure_ascii=False));return
    if args.action=='stop':
        if current:
            os.kill(current['identity']['pid'],signal.SIGTERM)
            for _ in range(150):
                if identity(current['identity']['pid']) is None:break
                time.sleep(.1)
            else:raise SystemExit('Owned demo did not stop; no blind kill')
        if PID.exists():PID.unlink()
        print('Stopped only sim-demo');return
    if current:raise SystemExit('sim-demo already running')
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        probe.bind((args.host,args.port))
    out=args.output or ROOT/('sim-demo-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    command=[str(PYTHON),str(ROOT/'mujoco_demo/scripts/sim_demo_main.py'),'--output',str(out),
             '--chassis-config',str(args.chassis_config),'--task-config',str(args.task_config),
             '--resolution',args.resolution,'--host',args.host,'--port',str(args.port)]
    with (out/'stdout.log').open('w') as stream:
        child=subprocess.Popen(command,cwd=ROOT/'mujoco_demo',env=dict(os.environ,MUJOCO_GL='egl',PYTHONDONTWRITEBYTECODE='1'),
                               stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    record={'identity':identity(child.pid),'output':str(out),'host':args.host,'port':args.port,'mode':'sim-demo'}
    PID.write_text(json.dumps(record,indent=2));print(json.dumps(record))


if __name__=='__main__':main()
