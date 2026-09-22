"""Prepare a pinned native build and copy model/graph assets from the known Python environment."""
import argparse
import pathlib
import shutil
import json
import hashlib
import mediapipe as mp
from google.protobuf import text_format
from mediapipe.framework import calculator_pb2

parser = argparse.ArgumentParser()
parser.add_argument('--root', type=pathlib.Path, required=True)
parser.add_argument('--opencv', required=True)
args = parser.parse_args()
assert mp.__version__ == '0.10.9', 'Use the reference MediaPipe 0.10.9 environment'
root = args.root.resolve()
source = root / 'ThirdParty/mediapipe-0.10.9'
bridge = source / 'mediapipe/examples/qt_bridge'
bridge.mkdir(parents=True, exist_ok=True)
for name in ('bridge.h', 'bridge.cc', 'BUILD'):
    shutil.copy2(root / 'mediapipe_bridge' / name, bridge / name)
workspace = source / 'WORKSPACE'
content = workspace.read_text(encoding='utf-8')
content = content.replace('path = "C:\\\\opencv\\\\build"', 'path = ' + json.dumps(args.opencv.replace('\\', '/')))
workspace.write_text(content, encoding='utf-8')
opencv = source / 'third_party/opencv_windows.BUILD'
content = opencv.read_text(encoding='utf-8').replace('OPENCV_VERSION = "3410"', 'OPENCV_VERSION = "4120"').replace('x64/vc15/', 'x64/vc16/')
opencv.write_text(content, encoding='utf-8')
assets = root / 'assets'
assets.mkdir(exist_ok=True)
package = pathlib.Path(mp.__file__).parent
hashes = {}
for folder in ('hand_landmark', 'palm_detection'):
    target = assets / 'mediapipe/modules' / folder
    target.mkdir(parents=True, exist_ok=True)
    for file in (package / 'modules' / folder).iterdir():
        if file.suffix in ('.tflite', '.txt', '.binarypb'):
            shutil.copy2(file, target / file.name)
            hashes[str(file.relative_to(package))] = hashlib.sha256(file.read_bytes()).hexdigest()
with mp.solutions.hands.Hands(model_complexity=0) as detector:
    config = calculator_pb2.CalculatorGraphConfig()
    config.ParseFromString(detector._graph.binary_config)
    graph_text = text_format.MessageToString(config)
    xnnpack = 'xnnpack {\n        }'
    assert graph_text.count(xnnpack) == 2, 'Expected palm and landmark XNNPACK delegates'
    graph_text = graph_text.replace(xnnpack, 'xnnpack {\n          num_threads: 4\n        }')
    (assets / 'hands_0_10_9.pbtxt').write_text(graph_text, encoding='utf-8')
(assets / 'source_hashes.json').write_text(json.dumps(hashes,indent=2), encoding='utf-8')
print('Prepared pinned MediaPipe build, expanded graph, and matching model assets.')
