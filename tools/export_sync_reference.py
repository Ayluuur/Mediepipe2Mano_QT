"""Generate differential fixtures directly from the current Python project."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import numpy as np

parser=argparse.ArgumentParser()
parser.add_argument('--source',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
source=args.source.resolve(); output=args.output.resolve()
sys.path.insert(0,str(source)); sys.path.insert(0,str(source/'tests'))
from mediapipe2mesh.config import load_config
from mediapipe2mesh.tracking.filters import OneEuroFilter
from mediapipe2mesh.tracking.depth import MultiHandDepthEstimator
from mediapipe2mesh.tracking.hand_state import HandState
from mediapipe2mesh.tracking.handedness import HandednessResolver
from mediapipe2mesh.interaction.button import DepthButton
from mediapipe2mesh.visualization import SceneMapper
from mediapipe2mesh.apps.common import update_hand_scene_position
from test_hand_spacing import HandSpacingTests, MP_TO_MANO

def serial(value):
    if isinstance(value,np.ndarray): return serial(value.tolist())
    if isinstance(value,(list,tuple)): return [serial(v) for v in value]
    if isinstance(value,dict): return {str(k):serial(v) for k,v in value.items()}
    if isinstance(value,(float,np.floating)) and not np.isfinite(value): return None
    if isinstance(value,np.generic): return value.item()
    return value

fixture={}
rng=np.random.default_rng(149)
fixture['filters']=[]
for options,shape in [({},(21,3)),({'median_window':3},(21,3)),
        ({'min_cutoff':1.5,'beta':.005,'median_window':3,'max_speed':500.},(1,2)),
        ({'min_cutoff':1.5,'beta':1.5,'median_window':3,'max_speed':500/300},(1,2))]:
    filt=OneEuroFilter(**options); frames=[]
    for i in range(220):
        point=np.zeros(shape) if i<20 else np.full(shape,200.)
        if i==10: point+=500
        if i==100: point[:]=np.nan
        if i>=200: point=rng.normal(size=shape)
        reset=i==200
        if reset: filt.reset()
        frames.append(dict(time=i/30,input=point,reset=reset,output=filt(point,i/30)))
    fixture['filters'].append(dict(options=options,frames=frames))

tracker=MultiHandDepthEstimator(load_config('interaction.json').depth_estimation)
fixture['depth_tracker']=[]
for i in range(240):
    t=i/30; request=i in (45,60,180)
    if request: tracker.request_calibration(t)
    sizes={'left':20.,'right':35.}
    if i<15: sizes.pop('right')
    if i>=30: sizes={'left':30.,'right':30.}
    if i>=150: sizes={'left':22.,'right':33.}
    if i in (40,41): sizes['left']=float('nan')
    if 145<=i<=152 or i>210: sizes={}
    actual=tracker.update({},t,640,480,palm_sizes=sizes)
    fixture['depth_tracker'].append(dict(time=t,request=request,sizes=sizes,output=actual,
        progress=[e.calibration_progress for e in tracker.estimators.values()],status=tracker.calibration_status(t)))

button=DepthButton([-150,0,350],90,60,20,10)
fixture['button']=[]
steps=[{}, {'left':[-150,0,320]}, {'left':[-150,0,380]},
       {'left':[-150,0,350],'right':[-140,0,350]}, {'right':[-140,0,350]}, {},
       {'left':[-150,0,400]}, {'left':[-150,0,370]}, {},
       {'left':[-150,0,350]}, {'left':[-94,0,350]}, {'left':[-90,0,350]},
       {'left':[float('nan'),0,350]}, {'right':[-300,0,320]}, {'right':[0,0,380]}, {},
       {'right':[-150,0,350]}, {'right':[-150,0,393]}, {'right':[-150,0,394]}, {}]
for points in steps:
    button.update(points)
    fixture['button'].append(dict(points=points,pressed=button.pressed,count=button.press_count,
        contacts=sorted(button.contacts),vertices=np.asarray(button.cap.vertices).copy(),color=np.asarray(button.cap.vertex_colors)[0].copy()))

fixture['presence']=[]
states={s:HandState(s,5,.7) for s in ('left','right')}
resolver=HandednessResolver(confirm_frames=3)
for i in range(100):
    raw=[dict(raw_side='left' if i<3 else 'right',wrist=np.array([.3,.5]),score=.9)]
    if 10<=i<12 or 20<=i<40: raw.append(dict(raw_side='right',wrist=np.array([.7,.5]),score=.8))
    if 40<=i<42 or 50<=i<67: raw=[]
    if 67<=i<70: raw[0]['wrist']=np.array([.8,.5])
    if i>=85:
        raw=[dict(raw_side='right',wrist=np.array([.7,.5]),score=.9),
             dict(raw_side='left',wrist=np.array([.3,.5]),score=.8)]
    if i==84:
        for state in states.values(): state.deactivate()
    result=resolver.resolve(raw,states,i/30)
    for side,d in result.items(): states[side].screen_wrist=d['wrist']; states[side].last_seen=i/30
    fixture['presence'].append(dict(time=i/30,raw=raw,resolved=result,order=list(result),reset=i==84,
        active=[s for s,st in states.items() if st.screen_wrist is not None]))

HandSpacingTests.setUpClass()
fixture['palm_scale']=[]
mapper=SceneMapper()
for reference,_ in HandSpacingTests.hands:
    correction={}; frames=[]
    for angle in (0,30,60,75,85,90,95,105,120,150,180):
        a=np.deg2rad(angle); rotation=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
        world=reference[MP_TO_MANO]@rotation.T
        for magnification in (1.,.5,2.):
            screen=np.zeros((21,3)); screen[:,:2]=(world[:,:2]+[-100,60])*magnification/[640,480]+.5
            scale=mapper.palm_scale(screen,world*.7/1000,reference,640,480,segment_corrections=correction)
            translation=mapper.screen_translation(screen,reference,640,480,75,mm_per_pixel=scale)
            frames.append(dict(screen=screen,world=world*.7/1000,scale=scale,corrections=dict(correction),translation=translation))
    fixture['palm_scale'].append(dict(reference=reference,frames=frames))

state=HandState('left',5,.7,load_config('interaction.json').tracking.position_filter)
reference=HandSpacingTests.hands[0][0]
state.keypoints=reference
fixture['scene_position']=dict(reference=reference,frames=[])
for i in range(60):
    scale=3 if i==10 else 1
    if 30<=i<40: scale=np.nan
    screen=np.full((21,3),.5); screen[0,0]=.7 if i<20 else .8
    if i==45: screen[:]=np.nan
    update_hand_scene_position(state,dict(screen=screen),mapper,200+i,640,480,i/30,mm_per_pixel=scale)
    fixture['scene_position']['frames'].append(dict(time=i/30,screen=screen,scale=scale,depth=200+i,
        translation=state.scene_translation.copy(),cached_scale=state.scene_mm_per_pixel))

(output/'tests/sync_reference.json').write_text(json.dumps(serial(fixture),allow_nan=False),encoding='utf-8')
paths=list((source/'mediapipe2mesh').rglob('*.py'))+list((source/'inverse_kinematics').glob('*.py'))+list((source/'configs').glob('*.json'))
manifest=dict(commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip(),
    files={str(p.relative_to(source)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
(output/'validation/python_source_snapshot.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print('Exported current Python differential fixtures and source hashes.')
