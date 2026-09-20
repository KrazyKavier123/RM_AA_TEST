#!/usr/bin/env python3
"""One FastAPI service for ROS/training monitoring and source configuration editing."""
import argparse
import os
from pathlib import Path

import uvicorn
from app import create_app


def workspace_default():
    if os.environ.get('RM_AA_WORKSPACE'):
        return Path(os.environ['RM_AA_WORKSPACE'])
    for path in [Path.cwd(), *Path(__file__).resolve().parents]:
        if (path / 'src/rm_aa_bringup').is_dir():
            return path
    return Path.cwd()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--workspace', type=Path, default=workspace_default())
    parser.add_argument('--ros', action='store_true', help='Subscribe to ROS2 telemetry, camera and gimbal topics')
    parser.add_argument('--demo', action='store_true', help='Generate clearly labelled demo metrics, no hardware')
    args, _ = parser.parse_known_args()
    here = Path(__file__).resolve()
    static = here.parents[1] / 'static'
    if not static.is_dir():
        static = here.parents[2] / 'share/rm_aa_webui/static'
    if not (args.workspace / 'src').is_dir():
        parser.error('--workspace must point to the source RM_AA workspace')
    app = create_app(args.workspace, static, args.ros, args.demo)
    # Single worker: telemetry history and the optional ROS node are process-local.
    uvicorn.run(app, host=args.host, port=args.port, log_level='info')


if __name__ == '__main__':
    main()
