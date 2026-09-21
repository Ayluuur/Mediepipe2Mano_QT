"""One-time conversion of the user's trusted MANO files and Python parity fixtures.

Python is only needed for conversion/testing, not by the native executable.
The source models use pickle despite their .npz suffix; never run on untrusted files.
"""
import argparse
import hashlib
import json
import pathlib
import struct
import sys
from types import SimpleNamespace

import numpy as np


def matrix(stream, value):
    value = np.asarray(value, dtype='<f8', order='C')
    if value.ndim == 1:
        value = value.reshape(1, -1)
    stream.write(struct.pack('<II', *value.shape))
    stream.write(value.tobytes())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=pathlib.Path, required=True)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.source))
    from mediapipe2mesh.ik.keypoints_to_mano import Keypoints2Mano
    from mediapipe2mesh.tracking.filters import OneEuroFilter
    from mediapipe2mesh.tracking.depth import RelativeDepthEstimator
    from mediapipe2mesh.interaction.components import rotation_between_vectors, palm_frame
    from mediapipe2mesh.interaction.components import InteractiveBall, BallController, PinchState
    from mediapipe2mesh.tracking.handedness import HandednessResolver
    from mediapipe2mesh.tracking.hand_state import HandState

    models = args.output / 'models'
    tests = args.output / 'tests'
    models.mkdir(parents=True, exist_ok=True)
    tests.mkdir(parents=True, exist_ok=True)
    manifest = {}
    fixtures = {'hands': [], 'filter': [], 'depth': [], 'rotations': []}
    rng = np.random.default_rng(1749)
    # Synthetic, nondegenerate hand. Tests do not depend on a camera or timing.
    world = np.array([[0, 0, 0], [-.02,-.01,0],[-.04,-.025,0],[-.05,-.04,0],[-.06,-.055,.005],
        [-.025,-.055,0],[-.025,-.08,.006],[-.025,-.10,.016],[-.025,-.11,.032],
        [0,-.06,0],[0,-.09,.002],[0,-.115,.012],[0,-.125,.024],
        [.022,-.055,0],[.022,-.08,.004],[.022,-.10,.015],[.022,-.11,.028],
        [.04,-.045,0],[.045,-.065,.007],[.048,-.08,.018],[.048,-.09,.03]])
    for side in ('left', 'right'):
        source = args.source / ('MANO_' + side.upper() + '.npz')
        data = np.load(source, allow_pickle=True)
        regressor = data['J_regressor'].toarray()
        with (models / ('MANO_' + side.upper() + '.bin')).open('wb') as stream:
            stream.write(b'M2MANO02')
            for value in (data['hands_components'], data['hands_mean'], regressor,
                          data['weights'], data['v_template'], data['f'], data['kintree_table'][0], data['posedirs'].reshape(778*3,135)):
                matrix(stream, value)
        manifest[source.name] = hashlib.sha256(source.read_bytes()).hexdigest()
        converter = Keypoints2Mano(str(source), side, max_iter=5, pose_smoothing=.7)
        fixtures['hands'].append({'side': side, 'rest_vertices': converter.mesh.verts.tolist(),
                                 'rest_keypoints': converter.mesh.keypoints.tolist(), 'frames': []})
        for index in range(5):
            points = world + rng.normal(0, .0003, world.shape)
            if side == 'left':
                points[:, 0] *= -1
            if index == 4:
                converter.reset()
            pose = converter.get_mano_params(points)
            fixtures['hands'][-1]['frames'].append(dict(
                reset=index == 4, world=points.tolist(), pose=pose.tolist(),
                vertices=converter.get_camera_oriented_vertices().tolist(),
                keypoints=converter.get_camera_oriented_keypoints().tolist(),
                palm_frame=palm_frame(points).tolist()))
    filt = OneEuroFilter()
    for index in range(25):
        timestamp = index / 30 if index < 20 else 3 + index / 30
        points = rng.normal(size=(21, 3))
        fixtures['filter'].append(dict(time=timestamp, input=points.tolist(), output=filt(points, timestamp).tolist()))
    depth = RelativeDepthEstimator()
    for index in range(65):
        points = world.copy()
        points[:, :2] = .5 + points[:, :2] * (2 if index < 35 else 3)
        timestamp = index / 30
        fixtures['depth'].append(dict(time=timestamp, input=points.tolist(),
            output=depth.update(points, timestamp, 640, 480), progress=depth.calibration_progress))
    for a, b in [([1,0,0],[0,1,0]),([1,0,0],[-1,0,0]),([0,0,0],[1,2,3]),
                  ([1,2,3],[-3,2,1]),([1,0,0],[1,0,0])]:
        fixtures['rotations'].append(dict(a=a,b=b,output=rotation_between_vectors(a,b).tolist()))
    fixtures['controller'] = []
    ball = InteractiveBall(35, (0, 0, 200))
    controller = BallController(ball, .35, 5, .8)
    states = {s: PinchState(s, .045, .055) for s in ('left', 'right')}
    # First hand acquired is right; rotation, clamping, release, and reacquisition.
    steps = [
        [(False, 0, [0,0,200]), (True, 1, [10,0,200])],
        [(False, 0, [0,0,200]), (True, 1, [20,10,210])],
        [(True, 2, [-10,10,210]), (True, 1, [20,10,210])],
        [(True, 2, [20,70,210]), (True, 1, [20,10,210])],
        [(True, 2, [20,10000,210]), (True, 1, [20,10,210])],
        [(True, 2, [20,10.00001,210]), (True, 1, [20,10,210])],
        [(True, 2, [20,15,210]), (False, 1, [20,10,210])],
        [(True, 2, [30,15,230]), (False, 1, [20,10,210])],
        [(False, 2, [30,15,230]), (False, 1, [20,10,210])],
        [(False, 2, [30,15,230]), (True, 3, [100,100,100])],
    ]
    for index, step in enumerate(steps):
        inputs=[]
        for side, (active, seq, point) in zip(('left', 'right'), step):
            state=states[side]
            state.pinching=state.controls_ball=active
            state.sequence=seq if active else None
            state.point=np.asarray(point,dtype=float)
            angle=index*.11
            state.hand_frame=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
            inputs.append(dict(active=active, sequence=seq, point=point, frame=state.hand_frame.tolist()))
        controller.update(states)
        fixtures['controller'].append(dict(inputs=inputs,center=ball.center.tolist(),rotation=ball.rotation.tolist(),scale=float(ball.scale)))
    fixtures['handedness'] = []
    states = {s: HandState(s, 5, .7) for s in ('left','right')}
    resolver = HandednessResolver(confirm_frames=3)
    frames=[(0,[('left',.3,.9)]),(.03,[('right',.31,.9)]),(.06,[('right',.32,.9)]),
            (.09,[('right',.33,.9)]),(.8,[('left',.25,.9),('right',.75,.8)]),
            (.83,[('right',.26,.8),('left',.74,.9)]),(.86,[('right',.27,.8),('right',.73,.7)]),
            (1.5,[('left',.8,.9),('left',.2,.5)]),(2.1,[])]
    for timestamp, detections in frames:
        raw=[dict(raw_side=s,wrist=np.array([x,.5]),score=score) for s,x,score in detections]
        result=resolver.resolve(raw,states,timestamp)
        fixtures['handedness'].append(dict(time=timestamp,raw=[dict(side=s,x=x,score=score) for s,x,score in detections],
            resolved=[dict(side=s,x=float(d['wrist'][0]),locked=d['label_was_stabilized']) for s,d in result.items()]))
        for side, detection in result.items():
            states[side].screen_wrist=detection['wrist']; states[side].last_seen=timestamp
    (tests / 'reference.json').write_text(json.dumps(fixtures), encoding='utf-8')
    (models / 'source_hashes.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print('Exported both MANO models and deterministic reference fixtures.')


if __name__ == '__main__':
    main()
