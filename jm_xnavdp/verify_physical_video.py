"""Decode the complete physical rollout and cross-check telemetry, not success animation."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
p=argparse.ArgumentParser(); p.add_argument('--run',type=Path,required=True); a=p.parse_args()
r=json.loads((a.run/'result.json').read_text()); m=json.loads((a.run/'manifest.json').read_text()); rows=json.loads((a.run/'telemetry.json').read_text())
assert r['actual_dynamic_H1'] and not r['root_writes_during_motion'] and r['source_world_unchanged']
review=a.run/'review'; review.mkdir(exist_ok=False)
video=a.run/f"{m['world']}_xnavdp_physical_720p.mp4"
cap=cv2.VideoCapture(str(video)); fps=cap.get(cv2.CAP_PROP_FPS)
count=0; invalid=[]; evidence=[]; ids={0,r['frames']//3,2*r['frames']//3,r['frames']-1}
while True:
    ok,frame=cap.read()
    if not ok: break
    if frame.shape!=(720,1280,3) or frame.mean()<1: invalid.append(count)
    if count in ids:
        path=review/f'decoded_{count:04d}.jpg'; cv2.imwrite(str(path),frame)
        evidence.append(str(path.resolve()))
    count+=1
cap.release()
positions=np.asarray([row['position_world'] for row in rows])
total=float(np.linalg.norm(np.diff(positions[:,:2],axis=0),axis=1).sum())
info={'video_integrity_pass':count==r['frames'] and abs(fps-24)<1e-3 and not invalid,
      'task_pass':r['pass'],'failure':r['failure'],'decoded_frames':count,'fps':fps,'duration_s':count/fps,
      'resolution':[1280,720],'invalid_frames':invalid,'measured_root_path_length_m':total,
      'measured_displacement_m':r['measured_displacement_m'],'evidence_frames':evidence,
      'actual_physics':True,'not_real_time_validation':True}
(review/'video_review.json').write_text(json.dumps(info,indent=2),encoding='utf-8')
print(json.dumps(info,indent=2))
if not info['video_integrity_pass']: raise SystemExit(1)
