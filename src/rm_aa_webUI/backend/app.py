import asyncio
from collections import deque
from contextlib import asynccontextmanager
import math
from pathlib import Path
import threading
import time
from typing import Any
from urllib.parse import urlsplit
import uuid

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, FiniteFloat

from config_files import ConfigFiles, edit_text


class Telemetry(BaseModel):
    source: str = Field(default='training', min_length=1, max_length=64)
    metrics: dict[str, FiniteFloat]
    timestamp_ms: FiniteFloat | None = None


class Sample(BaseModel):
    key: str
    value: FiniteFloat
    timestamp_ms: FiniteFloat


class TelemetryBatch(BaseModel):
    source: str = 'training'
    samples: list[Sample] = Field(max_length=4096)


class ConfigEdit(BaseModel):
    path: str
    revision: str = ''
    content: str = Field(max_length=500_000)
    changes: dict[str, Any] = Field(default_factory=dict)


class Hub:
    def __init__(self):
        self.lock = threading.Lock()
        self.samples = deque(maxlen=10000)
        self.sources = {}
        self.frames = {}
        self.sequence = 0
        self.epoch = uuid.uuid4().hex
        self.ros = '未启用'

    def log(self, source, metrics, timestamp=None):
        now = time.time() * 1000
        with self.lock:
            self.sources[source] = now
            for key, value in metrics.items():
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    continue
                self.sequence += 1
                self.samples.append(dict(id=self.sequence, key=f'{source}.{key}',
                                         t=now if timestamp is None else timestamp, v=float(value)))

    def image(self, kind, data):
        with self.lock:
            self.frames[kind] = (bytes(data), time.time() * 1000)

    def snapshot(self, after):
        with self.lock:
            return dict(epoch=self.epoch, cursor=self.sequence, now=time.time() * 1000,
                        samples=[sample for sample in self.samples if sample['id'] > after],
                        sources=dict(self.sources), ros=self.ros,
                        images={key: value[1] for key, value in self.frames.items()})


def create_app(workspace, static_dir=None, ros=False, demo=False):
    files, hub = ConfigFiles(workspace), Hub()
    stop = threading.Event()

    @asynccontextmanager
    async def lifespan(app):
        workers = []
        if ros:
            from ros_bridge import run_ros
            workers.append(threading.Thread(target=run_ros, args=(hub, stop), daemon=True))
        if demo:
            def run_demo():
                while not stop.wait(0.05):
                    t = time.time()
                    hub.log('demo', {'target_yaw': math.sin(t) * 0.2, 'target_pitch': math.cos(t * 0.6) * 0.08,
                                     'camera_fps': 60 + math.sin(t * 2), 'process_time_ms': 8 + math.cos(t)})
            workers.append(threading.Thread(target=run_demo, daemon=True))
        for worker in workers:
            worker.start()
        yield
        stop.set()
        for worker in workers:
            worker.join(timeout=2)

    app = FastAPI(title='RM_AA Monitor', lifespan=lifespan)
    app.state.hub, app.state.config_files = hub, files

    @app.middleware('http')
    async def same_origin(request: Request, call_next):
        origin = request.headers.get('origin')
        if origin and urlsplit(origin).netloc != request.headers.get('host'):
            return JSONResponse({'detail': '仅允许同源访问'}, status_code=403)
        return await call_next(request)

    @app.post('/api/telemetry')
    async def telemetry(batch: Telemetry | TelemetryBatch):
        if isinstance(batch, TelemetryBatch):
            for sample in batch.samples:
                hub.log(batch.source, {sample.key: sample.value}, sample.timestamp_ms)
            return {'accepted': len(batch.samples)}
        hub.log(batch.source, batch.metrics, batch.timestamp_ms)
        return {'accepted': len(batch.metrics)}

    @app.get('/api/status')
    async def status():
        state = hub.snapshot(hub.sequence)
        return {**state, 'workspace': str(files.root), 'demo': demo}

    @app.websocket('/ws')
    async def websocket(ws: WebSocket):
        origin = ws.headers.get('origin')
        if origin and urlsplit(origin).netloc != ws.headers.get('host'):
            await ws.close(code=1008)
            return
        await ws.accept()
        try:
            cursor = max(0, int(ws.query_params.get('after', '0')))
            if cursor > hub.sequence:
                cursor = 0
            while True:
                payload = hub.snapshot(cursor)
                await ws.send_json(payload)
                cursor = payload['cursor']
                await asyncio.sleep(0.1)
        except (WebSocketDisconnect, RuntimeError, OSError):
            pass

    @app.get('/api/frames/{kind}')
    async def frame(kind: str):
        with hub.lock:
            data = hub.frames.get(kind)
        if data is None:
            raise HTTPException(404, '尚无图像')
        return Response(data[0], media_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    @app.get('/api/configs')
    async def configs():
        return {'files': files.paths(), 'workspace': str(files.root)}

    def config_error(error):
        if isinstance(error, FileExistsError):
            raise HTTPException(409, str(error)) from error
        raise HTTPException(400, str(error)) from error

    @app.get('/api/config')
    async def config(path: str):
        try:
            return files.describe(path)
        except Exception as error:
            config_error(error)

    @app.post('/api/config/preview')
    async def preview(edit: ConfigEdit):
        try:
            files.resolve(edit.path)
            content = edit_text(edit.content, edit.changes, edit.path.endswith('.py'))
            return files.describe(edit.path, content)
        except Exception as error:
            config_error(error)

    @app.put('/api/config')
    async def save(edit: ConfigEdit):
        try:
            content = edit_text(edit.content, edit.changes, edit.path.endswith('.py'))
            result = files.save(edit.path, edit.revision, content)
            return {**result, 'message': '已写入文件。重启对应节点/程序后生效；launch 修改需重新启动 launch。'}
        except Exception as error:
            config_error(error)

    static = Path(static_dir) if static_dir else Path(__file__).resolve().parents[1] / 'static'
    app.mount('/static', StaticFiles(directory=static), name='static')

    @app.get('/')
    async def index():
        return FileResponse(static / 'index.html')

    return app
