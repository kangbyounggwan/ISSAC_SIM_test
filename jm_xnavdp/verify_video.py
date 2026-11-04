"""Decode every recorded frame and extract actual encoded evidence frames."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--run',type=Path,required=True)
args=p.parse_args()
result=json.loads((args.run/'result.json').read_text())
out=args.run/'review'
if out.exists(): raise RuntimeError('Review output already exists')
out.mkdir()
assert result['physics_stepped'] is False and result['robot_commands_sent']==0
assert result['robot_transform_changed'] is False and result['sources_unchanged'] is True
cap=cv2.VideoCapture(result['video'])
if not cap.isOpened(): raise RuntimeError('Cannot decode video')
fps=cap.get(cv2.CAP_PROP_FPS)
expected=result['frames']
sample_ids={0,expected//3,2*expected//3,expected-1}
count=0; failed=[]; previous=None; changed=0; samples=[]
while True:
    ok,frame=cap.read()
    if not ok: break
    if frame.shape != (720,1280,3) or frame.mean()<1: failed.append(count)
    if previous is not None and np.mean(np.abs(frame.astype(float)-previous.astype(float)))>.01:
        changed+=1
    if count in sample_ids:
        file=out/f'decoded_{count:04d}.jpg'
        cv2.imwrite(str(file),frame,[cv2.IMWRITE_JPEG_QUALITY,95])
        samples.append(str(file.resolve()))
    previous=frame
    count+=1
cap.release()
passed=count==expected and abs(fps-24)<1e-3 and not failed and changed>0
report={'pass':passed,'decoded_frames':count,'fps':fps,'duration_s':count/fps,
        'resolution':[1280,720],'invalid_frames':failed,'changed_frames':changed,
        'change_is_camera_and_debug_line_only':True,'robot_motion_claimed':False,
        'evidence_frames':samples}
(out/'video_review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
if not passed: raise SystemExit(1)
