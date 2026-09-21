#!/usr/bin/env python3
"""Linux ROS2 integration check: recorded video -> ROS -> HTTP."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import math
from websockets.sync.client import connect
from urllib.error import URLError
from urllib.request import urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', required=True, help='Absolute video directory or file')
    parser.add_argument('--config', help='Absolute perception YAML')
    args = parser.parse_args()
    if os.name != 'posix':
        raise RuntimeError('Run in the Linux ROS2 environment after sourcing install/setup.bash')
    workspace = Path(__file__).resolve().parents[1]
    output = workspace / 'output'
    output.mkdir(exist_ok=True)
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    with (output / 'ros_smoke.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'rm_aa_bringup', 'perception.launch.py', f'port:={port}',
             'source:=video', f'video:={args.video}'] + ([f'config:={args.config}'] if args.config else []),
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 30
            last = {}
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('Launch exited early; inspect output/ros_smoke.log')
                try:
                    with urlopen(f'http://127.0.0.1:{port}/api/status', timeout=1) as response:
                        last = json.load(response)
                    if 'vision' in last.get('sources', {}) and set(last.get('images', {})) >= {'image', 'mask'}:
                        break
                except (URLError, TimeoutError):
                    pass
                time.sleep(0.1)
            else:
                raise AssertionError(f'No complete C++ observation frame: {last}')
            metrics = {}
            with connect(f'ws://127.0.0.1:{port}/ws') as websocket:
                for _ in range(50):
                    packet = json.loads(websocket.recv(timeout=2))
                    metrics.update({sample['key']: sample['v'] for sample in packet['samples']})
                    if metrics.get('vision.sequence', 0) >= 10:
                        break
            assert metrics.get('vision.sequence', 0) >= 10, 'Need a recording with at least 10 processed frames'
            for route in ('image', 'mask'):
                with urlopen(f'http://127.0.0.1:{port}/api/frames/{route}', timeout=2) as response:
                    data = response.read()
                    assert response.headers['Content-Type'] == 'image/jpeg'
                    assert data[:2] == b'\xff\xd8' and len(data) > 100
            (output / 'ros_smoke.json').write_text(json.dumps({'status': last, 'metrics': metrics}, indent=2), encoding='utf-8')
            print('PASS: C++ video -> ROS -> FastAPI -> WebSocket metrics / HTTP JPEG')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)


if __name__ == '__main__':
    main()
