"""Compare native MediaPipe output against 0.10.9 on identical local GIF frames."""
import argparse
import ctypes as ct
import json
import os
from pathlib import Path
import time

import cv2
import mediapipe as mp
import numpy as np
from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument('--root', type=Path, required=True)
parser.add_argument('--gif', type=Path, required=True)
parser.add_argument('--opencv-bin', type=Path, default=Path('path/to/opencv/build/x64/vc16/bin'))
args = parser.parse_args()
root = args.root.resolve()
gif_path = args.gif.resolve()
assert mp.__version__ == '0.10.9'
dll_dirs = [os.add_dll_directory(str(args.opencv_bin.resolve())), os.add_dll_directory(str(root / 'bin'))]
os.chdir(root / 'assets')

class Hand(ct.Structure):
    _fields_ = [('side', ct.c_int32), ('score', ct.c_float), ('screen', ct.c_float * 63), ('world', ct.c_float * 63)]

dll = ct.CDLL(str(root / 'Bin/mediapipe_hands.dll'))
dll.m2m_create.argtypes = [ct.c_char_p, ct.c_int, ct.c_int, ct.c_int, ct.c_float, ct.c_float, ct.c_char_p, ct.c_int]
dll.m2m_create.restype = ct.c_void_p
dll.m2m_process.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_int, ct.c_int, ct.c_int, ct.c_int64, ct.POINTER(Hand), ct.c_int, ct.c_char_p, ct.c_int]
dll.m2m_process.restype = ct.c_int
dll.m2m_destroy.argtypes = [ct.c_void_p]
gif = Image.open(gif_path)
frames = [np.zeros((338,296,3),dtype=np.uint8)]
for index in range(0, min(gif.n_frames, 300), 15):
    gif.seek(index)
    frames.append(np.ascontiguousarray(np.array(gif.convert('RGB'))[20:358,304:600]))
frames.append(np.zeros_like(frames[0]))
reports=[]
for complexity in (0,1):
    error=ct.create_string_buffer(4096)
    handle=dll.m2m_create(str(root/'assets/hands_0_10_9.pbtxt').encode(),complexity,2,4,.5,.6,error,len(error))
    if not handle:
        raise RuntimeError(error.value.decode())
    worst_screen=worst_world=worst_score=0.
    detected=0
    times=[]
    try:
        with mp.solutions.hands.Hands(model_complexity=complexity,max_num_hands=2,min_detection_confidence=.5,min_tracking_confidence=.6) as reference:
            for index, frame in enumerate(frames):
                output=(Hand*2)()
                started=time.perf_counter()
                count=dll.m2m_process(handle,frame.ctypes.data,frame.shape[1],frame.shape[0],frame.strides[0],(index+1)*33333,output,2,error,len(error))
                times.append((time.perf_counter()-started)*1000)
                if count<0:
                    raise RuntimeError(error.value.decode())
                expected=reference.process(frame)
                assert count==len(expected.multi_hand_landmarks or []), (index,'hand count mismatch')
                detected+=count
                for hand_index in range(count):
                    actual=output[hand_index]
                    classification=expected.multi_handedness[hand_index].classification[0]
                    assert actual.side==(0 if classification.label=='Left' else 1)
                    worst_score=max(worst_score,abs(actual.score-classification.score))
                    a=np.ctypeslib.as_array(actual.screen).reshape(21,3)
                    b=np.array([[p.x,p.y,p.z] for p in expected.multi_hand_landmarks[hand_index].landmark])
                    worst_screen=max(worst_screen,float(np.max(np.abs(a-b))))
                    a=np.ctypeslib.as_array(actual.world).reshape(21,3)
                    b=np.array([[p.x,p.y,p.z] for p in expected.multi_hand_world_landmarks[hand_index].landmark])
                    worst_world=max(worst_world,float(np.max(np.abs(a-b))))
    finally:
        dll.m2m_destroy(handle)
    report=dict(complexity=complexity,frames=len(frames),detected_hands=detected,max_screen_error=worst_screen,
                max_world_error_m=worst_world,max_score_error=worst_score,mean_native_ms=float(np.mean(times[1:])))
    reports.append(report)
    print(json.dumps(report),flush=True)
    assert detected>0, 'Fixture did not exercise a detected hand'
    assert worst_screen<1e-5 and worst_world<1e-6 and worst_score<1e-5, 'Detector parity tolerance exceeded'
(root/'validation').mkdir(parents=True,exist_ok=True)
(root/'validation/detector.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
