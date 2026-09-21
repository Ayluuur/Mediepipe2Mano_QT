"""Render the same deterministic scene with the unchanged Python implementation."""
import argparse
import pathlib
import sys
import json

import cv2
import numpy as np
import open3d as o3d

parser = argparse.ArgumentParser()
parser.add_argument('--source', type=pathlib.Path, required=True)
parser.add_argument('--output', type=pathlib.Path, required=True)
parser.add_argument('--mode', choices=('viewer', 'interaction'), default='interaction')
args = parser.parse_args()
sys.path.insert(0, str(args.source.resolve()))
from mediapipe2mesh.config import load_config
from mediapipe2mesh.tracking.hand_state import HandState
from mediapipe2mesh.visualization.scene import SceneMapper, create_scene_bounds, configure_view
from mediapipe2mesh.interaction.components import InteractiveBall
from mediapipe2mesh.interaction.button import DepthButton

config = load_config(args.mode + '.json')
vis = o3d.visualization.Visualizer()
assert vis.create_window(width=1920, height=1080, left=50, top=50, visible=False)
try:
    bounds = create_scene_bounds(vis, *([ -80, 80 ] if args.mode == 'interaction' else [-60, 60]))
    configure_view(vis, config.viewer.initial_view)
    if args.mode == 'interaction':
        ball = InteractiveBall(config.ball.radius, config.ball.center)
        ball.add_to(vis)
        button = DepthButton(config.button.center, config.button.width, config.button.height,
                             config.button.travel, config.button.tip_radius)
        button.add_to(vis)
    states = []
    for index, side in enumerate(('left', 'right')):
        state = HandState(side, config.mano.iterations, config.mano.pose_smoothing)
        points = state.converter.get_camera_oriented_vertices()
        vertices = SceneMapper().hand_vertices(points, np.array([.3 if index == 0 else .7, .5]), config.depth_estimation.reference_depth)
        state.mesh.vertices = o3d.utility.Vector3dVector(vertices)
        state.mesh.compute_vertex_normals()
        vis.add_geometry(state.mesh, reset_bounding_box=False)
        states.append(state)
    for _ in range(3):
        vis.poll_events()
        vis.update_renderer()
    args.output.mkdir(parents=True, exist_ok=True)
    image_path = args.output / ('python_' + args.mode + '.png')
    vis.capture_screen_image(str(image_path), do_render=True)
finally:
    vis.destroy_window()
cpp = cv2.imread(str(args.output / ('cpp_' + args.mode + '.png')))
python = cv2.imread(str(image_path))
if cpp is None or cpp.shape != python.shape:
    raise RuntimeError('Missing or dimension-mismatched C++ capture')
diff = np.abs(cpp.astype(np.int16) - python.astype(np.int16))
report = dict(mode=args.mode, shape=list(cpp.shape), max_channel_error=int(diff.max()),
              mean_channel_error=float(diff.mean()), differing_pixels=int(np.count_nonzero(diff.max(axis=2))))
camera=np.zeros((config.camera.height,config.camera.width,3),dtype=np.uint8)
if args.mode=='interaction':
    cv2.putText(camera,'FPS 0.0  pinch enter/exit: {:.0f}/{:.0f}mm'.format(config.pinch.enter_distance*1000,config.pinch.exit_distance*1000),
                (12,28),cv2.FONT_HERSHEY_SIMPLEX,.58,(80,255,80),2,cv2.LINE_AA)
    cv2.putText(camera,'Ball scale 1.00x  [1 front / 3 depth]',(12,54),cv2.FONT_HERSHEY_SIMPLEX,.52,(220,220,220),1,cv2.LINE_AA)
    cv2.putText(camera,'Input flip: {}  label map: {}'.format('ON' if config.camera.mirror else 'OFF',config.tracking.handedness_map.upper()),
                (12,78),cv2.FONT_HERSHEY_SIMPLEX,.48,(220,220,220),1,cv2.LINE_AA)
else:
    cv2.putText(camera,'FPS 0.0',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(80,255,80),2,cv2.LINE_AA)
    cv2.putText(camera,'Open3D view: {}  [1 front / 3 depth]'.format(config.viewer.initial_view.upper()),
                (12,54),cv2.FONT_HERSHEY_SIMPLEX,.52,(210,210,210),1,cv2.LINE_AA)
if config.depth_estimation.enabled:
    cv2.putText(camera,'ENTER: calibrate depth after 3s',(12,camera.shape[0]-18),
                cv2.FONT_HERSHEY_SIMPLEX,.52,(80,255,255),1,cv2.LINE_AA)
if args.mode=='interaction':
    cv2.putText(camera,'Button: READY  presses: 0',(12,102),
                cv2.FONT_HERSHEY_SIMPLEX,.52,(220,220,220),2,cv2.LINE_AA)
cv2.imwrite(str(args.output/('python_'+args.mode+'.png.camera.png')),camera)
cpp_camera=cv2.imread(str(args.output/('cpp_'+args.mode+'.png.camera.png')))
camera_diff=np.abs(cpp_camera.astype(np.int16)-camera.astype(np.int16))
report['camera_max_channel_error']=int(camera_diff.max())
report['camera_differing_pixels']=int(np.count_nonzero(camera_diff.max(axis=2)))
(args.output / ('render_' + args.mode + '.json')).write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report))
if diff.max() != 0 or camera_diff.max()!=0:
    raise SystemExit(1)
