import importlib.util
import json
from pathlib import Path
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / 'src' / 'rm_aa_bringup'
SPEC = importlib.util.spec_from_file_location('debug_web', BRINGUP / 'scripts' / 'debug_web.py')
web = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(web)


class WebTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.state = web.State()
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), web.handler_for(cls.state, BRINGUP / 'web'))
        cls.server.daemon_threads = True
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f'http://127.0.0.1:{cls.server.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def test_read_only_methods(self):
        with self.assertRaises(HTTPError) as error:
            urlopen(Request(self.base + '/api/status', data=b'{}', method='POST'), timeout=2)
        self.assertEqual(error.exception.code, 405)

    def test_snapshot_and_stale_state(self):
        self.state.set_telemetry(json.dumps({'sequence': 27, 'detections': []}))
        with urlopen(self.base + '/api/status', timeout=2) as response:
            value = json.load(response)
        self.assertTrue(value['online'])
        self.assertEqual(value['sequence'], 27)
        with self.state.lock:
            self.state.updated = time.monotonic() - 3
        self.assertFalse(self.state.snapshot()['online'])

    def test_invalid_telemetry_preserves_previous(self):
        self.state.set_telemetry('{"sequence":12}')
        for invalid in ('[]', 'NaN', '{"x":NaN}', '{broken'):
            with self.assertRaises(ValueError):
                self.state.set_telemetry(invalid)
        self.assertEqual(self.state.snapshot()['sequence'], 12)

    def test_image_and_mime(self):
        payload = b'\xff\xd8\xff\xd9'
        self.state.set_frame('image', payload)
        with urlopen(self.base + '/frame.jpg?timestamp=1', timeout=2) as response:
            self.assertEqual(response.headers['Content-Type'], 'image/jpeg')
            self.assertEqual(response.read(), payload)

    def test_no_directory_traversal(self):
        for path in ('/../package.xml', '/%2e%2e/package.xml', '/setting/foo'):
            with self.assertRaises(HTTPError) as error:
                urlopen(self.base + path, timeout=2)
            self.assertEqual(error.exception.code, 404)

    def test_assets_available(self):
        for path in ('/', '/app.js', '/style.css'):
            with urlopen(self.base + path, timeout=2) as response:
                self.assertEqual(response.status, 200)
                self.assertGreater(len(response.read()), 100)


if __name__ == '__main__':
    unittest.main()
