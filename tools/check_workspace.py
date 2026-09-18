#!/usr/bin/env python3
"""Source/config/model checks; explicitly does not replace a C++/ROS2 build."""
import ast
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    results = []
    for path in list((ROOT / 'src').rglob('*.py')) + list((ROOT / 'tools').glob('*.py')) + list((ROOT / 'tests').glob('*.py')):
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    results.append('Python AST')
    for name in ('rm_aa_vision', 'rm_aa_bringup', 'rm_aa_control'):
        package = ROOT / 'src' / name
        assert ET.parse(package / 'package.xml').getroot().findtext('name') == name
        assert (package / 'CMakeLists.txt').is_file()
    results.append('ROS package XML and CMake entry points')
    model_dir = ROOT / 'src' / 'rm_aa_vision' / 'models'
    digest = hashlib.sha256((model_dir / 'mlp.onnx').read_bytes()).hexdigest()
    assert digest == 'b3250176ea8fdbe6d1585be954363757ff6a8245a81e8797f16cea601976d47a'
    labels = (model_dir / 'label.txt').read_text().splitlines()
    assert labels == ['1', '2', '3', '4', '5', 'outpost', 'guard', 'base', 'negative']
    results.append('Bundled model SHA256 and label ordering')
    import yaml
    config = yaml.safe_load((ROOT / 'src/rm_aa_bringup/config/perception.yaml').read_text())['perception']['ros__parameters']
    assert len(config['calibration']['k']) == 9
    assert all(isinstance(value, float) for value in config['calibration']['k'])
    assert config['dimensions']['large_width_m'] > config['dimensions']['small_width_m'] > 0
    assert config['source']['mode'] == 'synthetic'
    control = yaml.safe_load((ROOT / 'src/rm_aa_bringup/config/control.yaml').read_text())['camera_control']['ros__parameters']
    assert control['observations_topic'] == '/observations'
    assert 1 <= control['tracker']['plate_count'] <= 16
    assert control['tracker']['min_radius_m'] <= control['tracker']['radius_m'] <= control['tracker']['max_radius_m']
    assert control['mpc']['dt_s'] > 0 and control['mpc']['horizon'] >= 2
    results.append('ROS parameter YAML')
    import cv2
    import numpy as np
    net = cv2.dnn.readNetFromONNX(str(model_dir / 'mlp.onnx'))
    net.setInput(np.zeros((1, 1, 28, 20), dtype=np.float32))
    output = net.forward()
    assert output.shape == (1, 9) and np.isfinite(output).all()
    layers = [{'name': name, 'type': net.getLayer(net.getLayerId(name)).type} for name in net.getLayerNames()]
    results.append('Actual OpenCV ONNX forward: [1,1,28,20] -> [1,9]')
    report = {'passed': results, 'opencv': cv2.__version__, 'model_sha256': digest,
              'model_layers': layers, 'cpp_ros_build': 'not performed by this script'}
    output_dir = ROOT / 'output'
    output_dir.mkdir(exist_ok=True)
    (output_dir / 'source_checks.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(f'CHECK FAILED: {error}', file=sys.stderr)
        sys.exit(1)
