#!/usr/bin/env python3
"""Read-only ROS telemetry HTTP bridge. --demo is a UI fixture, not the C++ pipeline."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.telemetry = {}
        self.updated = None
        self.frames = {'image': b'', 'mask': b''}

    def set_telemetry(self, text):
        value = json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
        if not isinstance(value, dict):
            raise ValueError('Telemetry must be a JSON object')
        with self.lock:
            self.telemetry = value
            self.updated = time.monotonic()

    def snapshot(self):
        with self.lock:
            age = None if self.updated is None else time.monotonic() - self.updated
            return {**self.telemetry, 'online': age is not None and age < 2.0,
                    'telemetry_age_s': age}

    def set_frame(self, name, data):
        with self.lock:
            self.frames[name] = bytes(data)

    def frame(self, name):
        with self.lock:
            return self.frames[name]


def handler_for(state, web_root):
    assets = {'/': ('index.html', 'text/html; charset=utf-8'),
              '/app.js': ('app.js', 'application/javascript; charset=utf-8'),
              '/style.css': ('style.css', 'text/css; charset=utf-8')}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def send(self, code, content_type, body):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob:; object-src 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/status':
                self.send(200, 'application/json', json.dumps(state.snapshot(), allow_nan=False).encode())
            elif path in ('/frame.jpg', '/mask.jpg'):
                body = state.frame('image' if path == '/frame.jpg' else 'mask')
                self.send(200 if body else 503, 'image/jpeg' if body else 'text/plain', body or b'Waiting for camera')
            elif path in assets:
                name, mime = assets[path]
                self.send(200, mime, (web_root / name).read_bytes())
            else:
                self.send(404, 'text/plain', b'Not found')

        def do_POST(self):
            self.send(405, 'text/plain', b'Read-only diagnostics')

    return Handler


def run_ros(state, stop, ros_args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    rclpy.init(args=ros_args)
    node = Node('debug_web')
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT)

    def telemetry(message):
        try:
            state.set_telemetry(message.data)
        except (ValueError, TypeError) as error:
            node.get_logger().warning(f'Rejected telemetry: {error}')

    # Hold references for the lifetime of the node.
    subscriptions = [
        node.create_subscription(String, '/telemetry', telemetry, qos),
        node.create_subscription(CompressedImage, '/debug/image/compressed', lambda m: state.set_frame('image', m.data), qos),
        node.create_subscription(CompressedImage, '/debug/mask/compressed', lambda m: state.set_frame('mask', m.data), qos),
    ]
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_demo(state, stop):
    import cv2
    import numpy as np
    index = 0
    while not stop.is_set():
        image = np.full((480, 800, 3), 18, dtype=np.uint8)
        offset = int(65 * np.sin(index / 30))
        for x in (260 + offset, 420 + offset):
            cv2.rectangle(image, (x, 205), (x + 8, 270), (255, 180, 180), -1)
        cv2.rectangle(image, (260 + offset, 205), (428 + offset, 270), (50, 200, 255), 2)
        cv2.putText(image, 'UI FIXTURE - NOT LIVE PERCEPTION', (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 1)
        mask = cv2.threshold(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 100, 255, cv2.THRESH_BINARY)[1]
        state.set_frame('image', cv2.imencode('.jpg', image)[1].tobytes())
        state.set_frame('mask', cv2.imencode('.jpg', mask)[1].tobytes())
        state.set_telemetry(json.dumps({
            'schema': 1, 'mode': 'web_fixture', 'sequence': index, 'capture_fps': None,
            'processing_fps': None, 'processing_ms': None, 'frame_age_ms': None,
            'calibration_valid': False, 'dropped': 0, 'tracker_resets': 0,
            'output_kind': 'observation_only', 'detections': [], 'image_tracks': [],
        }))
        index += 1
        stop.wait(0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--web-root', type=Path, default=Path(__file__).resolve().parents[1] / 'web')
    parser.add_argument('--demo', action='store_true')
    args, ros_args = parser.parse_known_args()
    if not (1 <= args.port <= 65535):
        parser.error('Port outside 1..65535')
    for name in ('index.html', 'app.js', 'style.css'):
        if not (args.web_root / name).is_file():
            parser.error(f'Missing web asset: {name}')
    if not args.demo:
        try:
            import rclpy  # noqa: F401
        except ImportError:
            parser.error('ROS2 is unavailable. Source the ROS setup or use --demo for a UI fixture.')
    state, stop = State(), threading.Event()
    server = ThreadingHTTPServer((args.host, args.port), handler_for(state, args.web_root))
    server.daemon_threads = True

    def worker():
        try:
            if args.demo:
                run_demo(state, stop)
            else:
                run_ros(state, stop, ros_args)
        except Exception as error:
            print(f'Bridge stopped: {error}', flush=True)
            stop.set()
            server.shutdown()

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    print(f'Diagnostics: http://{args.host}:{args.port}', flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        thread.join(timeout=2)


if __name__ == '__main__':
    main()
