"""Run simulation only; no SDK imports and no strict-mode process changes."""
import argparse
import json
import os
from pathlib import Path
import sys
os.environ.setdefault('MUJOCO_GL','egl')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from aiohttp import web
from vln_mujoco.sim_demo_runtime import DemoApp

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--chassis-config',type=Path,required=True)
parser.add_argument('--task-config',type=Path,required=True)
parser.add_argument('--resolution',default='960x540',choices=['960x540','640x360'])
parser.add_argument('--host',default='127.0.0.1',help='HTTP/WebSocket bind address; use 0.0.0.0 for LAN access')
parser.add_argument('--port',type=int,default=8091,help='HTTP/WebSocket port')
args=parser.parse_args()
resolution=tuple(map(int,args.resolution.split('x')))
server=DemoApp(args.chassis_config,args.output,json.loads(args.task_config.read_text()),resolution)
web.run_app(server.app(),host=args.host,port=args.port,print=None,shutdown_timeout=2.)
