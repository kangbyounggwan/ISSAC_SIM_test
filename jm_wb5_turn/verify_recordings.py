"""Decode every recorded frame and report resolution, duration and telemetry coverage."""
import argparse
import json
from pathlib import Path
import cv2

p=argparse.ArgumentParser()
p.add_argument('root',type=Path)
args=p.parse_args()
reports={}
for path in sorted(args.root.glob('*/turn_720p.mp4')):
    rows=json.loads((path.parent/'telemetry.json').read_text())
    cap=cv2.VideoCapture(str(path))
    fps=cap.get(cv2.CAP_PROP_FPS)
    reported=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames=0
    dark=[]
    dimensions=set()
    while True:
        ok,frame=cap.read()
        if not ok: break
        dimensions.add((frame.shape[1],frame.shape[0]))
        if float(frame[100:].mean())<4: dark.append(frames)
        frames+=1
    cap.release()
    integral=sum(r['command_applied'][2]*.02 for r in rows[1:])
    telemetry_ok=all(abs(rows[i]['time_s']-rows[i-1]['time_s']-.02)<1e-6 for i in range(1,len(rows)))
    report=dict(file=str(path.resolve()),frames_decoded=frames,frames_reported=reported,
        fps=fps,duration_s=frames/fps,resolutions=list(dimensions),dark_frames=dark,
        telemetry_rows=len(rows),telemetry_continuous_50hz=telemetry_ok,
        applied_yaw_command_integral_rad=integral,
        passed=(frames==reported and dimensions=={(1280,720)} and fps==25 and not dark and telemetry_ok))
    reports[path.parent.name]=report
(args.root/'video_qa.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
print(json.dumps(reports,indent=2))
if not reports or not all(r['passed'] for r in reports.values()): raise SystemExit(1)
