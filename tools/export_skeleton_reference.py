"""Export current Python FK, retargeting, and bone API differential fixtures."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

parser = argparse.ArgumentParser()
parser.add_argument('--source', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
sys.path.insert(0, str(args.source.resolve()))
from mediapipe2mesh.ik import Keypoints2Mano

mapping = [0,13,14,15,20,1,2,3,16,4,5,6,17,10,11,12,19,7,8,9,18]
result = dict(models=[], hands=[])
rng = np.random.default_rng(21)
rotation = Rotation.from_rotvec([.4,-.7,.3]).as_matrix()
for side in ('left','right'):
    path = args.source / ('MANO_'+side.upper()+'.npz')
    source = Keypoints2Mano(path, side, mirrored_input=False)
    for _ in range(3):
        pose = rng.normal(0,.25,(16,3))
        source.mesh.set_params(pose_abs=pose)
        result['models'].append(dict(side=side, pose=pose.tolist(),
            vertices=source.mesh.verts.tolist(), keypoints=source.mesh.keypoints.tolist(),
            transforms=source.mesh.joint_transforms.tolist()))
    for mirrored in (False,True):
        for n_pose in (17,45):
            hand = Keypoints2Mano(path,side,pose_smoothing=1,n_pose=n_pose,mirrored_input=mirrored)
            for amount in (0.,.5,1.):
                pose = np.zeros((16,3)); pose[1:,2] = amount
                source.mesh.set_params(pose_abs=pose)
                target = source.mesh.keypoints @ rotation.T @ np.diag([-1 if mirrored else 1,1,1])
                world = target[mapping]/1000 + [.2,-.1,.4]
                hand.reset()
                params = hand.get_mano_params(world)
                result['hands'].append(dict(side=side,mirrored=mirrored,n_pose=n_pose,amount=amount,
                    world=world.tolist(),pose=params.tolist(),target=(target-target[0]).tolist(),
                    vertices=hand.get_camera_oriented_vertices().tolist(),
                    keypoints=hand.get_camera_oriented_keypoints().tolist(),
                    faces=hand.get_faces(True).tolist(),
                    camera=hand.get_skeleton().to_dict(),model=hand.get_skeleton('model').to_dict(),
                    scene=hand.get_skeleton().to_scene([20,-30,100]).to_dict()))
(args.output/'tests/skeleton_reference.json').write_text(json.dumps(result,allow_nan=False),encoding='utf-8')
print(f'Exported {len(result["models"])} FK cases and {len(result["hands"])} skeleton/IK cases.')
