from pathlib import Path
import sys

import pytest
from fastapi.testclient import TestClient

UI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(UI / 'backend'))
from app import create_app
from config_files import fields_for


@pytest.fixture
def workspace(tmp_path):
    config = tmp_path / 'src/example/config'
    config.mkdir(parents=True)
    (config / 'control.yaml').write_text('# retain this comment\nnode:\n  gain: 1.5 # tuned\n  count: 4\n  active: false\n  title: "相机"\n', encoding='utf-8')
    launch = tmp_path / 'src/example/launch'
    launch.mkdir()
    (launch / 'test.launch.py').write_text("# 启动参数\nfrom launch.actions import DeclareLaunchArgument\nargument = DeclareLaunchArgument('port', default_value='8080')\n", encoding='utf-8')
    vision = tmp_path / 'src/example/config'
    (vision / 'camera.yaml').write_text('%YAML:1.0\n---\n# measured matrix\nK: !!opencv-matrix\n  rows: 1\n  cols: 2\n  dt: d\n  data: [ 1., 2. ]\n', encoding='utf-8')
    return tmp_path


@pytest.fixture
def client(workspace):
    with TestClient(create_app(workspace, UI / 'static')) as client:
        yield client


def document(client, path='src/example/config/control.yaml'):
    response = client.get('/api/config', params={'path': path})
    assert response.status_code == 200, response.text
    return response.json()


def save(client, doc, **changes):
    return client.put('/api/config', json={'path': doc['path'], 'revision': doc['revision'],
                                          'content': doc['content'], 'changes': changes})


def test_ui_and_source_catalog(client):
    assert client.get('/').status_code == 200
    assert 'WebSocket' in client.get('/static/app.js').text
    files = client.get('/api/configs').json()['files']
    assert len(files) == 3
    assert client.get('/api/config', params={'path': '../../outside.yaml'}).status_code == 400


def test_yaml_numeric_save_retains_comments_and_other_values(client, workspace):
    doc = document(client)
    response = save(client, doc, **{'/node/gain': 2.25})
    assert response.status_code == 200, response.text
    text = (workspace / doc['path']).read_text(encoding='utf-8')
    assert 'gain: 2.25 # tuned' in text and '# retain this comment' in text
    assert 'count: 4' in text and 'title: "相机"' in text
    assert '重启' in response.json()['message']


def test_opencv_matrix_keeps_directive_and_tag(client, workspace):
    doc = document(client, 'src/example/config/camera.yaml')
    response = save(client, doc, **{'/K/data/0': 123.5})
    assert response.status_code == 200, response.text
    text = (workspace / doc['path']).read_text(encoding='utf-8')
    assert text.startswith('%YAML:1.0') and '!!opencv-matrix' in text
    assert '[ 123.5, 2. ]' in text


def test_launch_default_save_retains_python_and_comments(client, workspace):
    doc = document(client, 'src/example/launch/test.launch.py')
    field = doc['fields'][0]
    response = save(client, doc, **{field['key']: '8088'})
    assert response.status_code == 200, response.text
    text = (workspace / doc['path']).read_text(encoding='utf-8')
    assert "default_value='8088'" in text and '# 启动参数' in text


def test_invalid_syntax_or_type_does_not_write(client, workspace):
    doc = document(client)
    assert save(client, doc, **{'/node/count': 1.5}).status_code == 400
    assert save(client, doc, **{'/node/gain': 'not a number'}).status_code == 400
    response = client.put('/api/config', json={**{k: doc[k] for k in ('path', 'revision')}, 'content': 'node: [unclosed'})
    assert response.status_code == 400
    assert (workspace / doc['path']).read_bytes().decode('utf-8') == doc['content']


def test_external_edit_conflict(client, workspace):
    doc = document(client)
    path = workspace / doc['path']
    path.write_text(doc['content'] + '# IDE change\n', encoding='utf-8')
    assert save(client, doc, **{'/node/gain': 9.0}).status_code == 409
    assert '# IDE change' in path.read_text(encoding='utf-8')


def test_preview_does_not_save(client, workspace):
    doc = document(client)
    response = client.post('/api/config/preview', json={'path': doc['path'], 'content': doc['content'], 'changes': {'/node/gain': 3.0}})
    assert response.status_code == 200
    assert 'gain: 3.0' in response.json()['content']
    assert (workspace / doc['path']).read_bytes().decode('utf-8') == doc['content']


def test_http_to_websocket_history_and_resume(client):
    assert client.post('/api/telemetry', json={'source': 'ros', 'metrics': {'yaw': .2}}).status_code == 200
    with client.websocket_connect('/ws') as socket:
        first = socket.receive_json()
        assert first['samples'][0]['key'] == 'ros.yaw'
        after = first['cursor']
    client.post('/api/telemetry', json={'source': 'ros', 'samples': [
        {'key': 'yaw', 'value': .3, 'timestamp_ms': 1000}, {'key': 'yaw', 'value': .4, 'timestamp_ms': 1010}]})
    with client.websocket_connect(f'/ws?after={after}') as socket:
        packet = socket.receive_json()
        assert [p['v'] for p in packet['samples']] == [.3, .4]
        assert [p['t'] for p in packet['samples']] == [1000, 1010]


def test_multiple_clients_receive_same_samples(client):
    with client.websocket_connect('/ws') as a, client.websocket_connect('/ws') as b:
        a.receive_json(); b.receive_json()
        client.post('/api/telemetry', json={'metrics': {'fps': 60}})
        for socket in (a, b):
            for _ in range(10):
                packet = socket.receive_json()
                if packet['samples']:
                    break
            assert packet['samples'][0]['v'] == 60


def test_frame_and_finite_input(client):
    client.app.state.hub.image('image', b'jpeg-test')
    assert client.get('/api/frames/image').content == b'jpeg-test'
    assert client.get('/api/frames/missing').status_code == 404
    assert client.post('/api/telemetry', json={'metrics': {'yaw': 'NaN'}}).status_code == 422
    assert client.put('/api/config', headers={'Origin': 'https://unrelated.example'}, json={}).status_code == 403


def test_repository_configs_parse():
    from config_files import ConfigFiles
    files = ConfigFiles(UI.parents[1])
    for path in files.paths():
        assert files.describe(path)['path'] == path
