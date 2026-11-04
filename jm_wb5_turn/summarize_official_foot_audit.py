"""Recount official reference steps, excluding takeoffs before the turn window."""
import argparse
import json
from pathlib import Path
import sys
from statistics import mean
import cv2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from jm_wb5_turn.footstep_metrics import foot_events,angle_travel,norm_xy,THRESHOLDS

p=argparse.ArgumentParser()
p.add_argument('root',type=Path)
args=p.parse_args()
reports={}
for file in sorted(args.root.glob('official_foot_*/telemetry.json')):
    rows=json.loads(file.read_text())
    result=json.loads((file.parent/'result.json').read_text())
    turn=[r for r in rows if 2.<=r['time_s']<=12.]
    brake=[r for r in rows if r['time_s']>=12.]
    events=[foot_events(rows,k,2.,12.) for k in range(2)]
    stop_events=[foot_events(rows,k,12.,14.) for k in range(2)]
    cap=cv2.VideoCapture(str(file.parent/'turn_720p.mp4'))
    fps=cap.get(cv2.CAP_PROP_FPS);reported=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    dark=[];dim=set();n=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        dim.add((frame.shape[1],frame.shape[0]))
        if float(frame[100:].mean())<4:dark.append(n)
        n+=1
    cap.release()
    report=dict(thresholds=THRESHOLDS,
        turn_window_s=[2,12],initial_airborne_at_2s_excluded=True,
        verified_lift_landing_counts=[sum(e['verified_lift_landing'] for e in es) for es in events],
        foot_events=events,stop_lift_landing_counts=[sum(e['verified_lift_landing'] for e in es) for es in stop_events],
        pelvis_turn_deg=angle_travel(turn,'yaw_deg'),
        torso_turn_deg=angle_travel(turn,'torso_yaw_world_deg'),
        foot_turn_deg=[angle_travel(turn,'foot_yaw_deg',k) for k in range(2)],
        max_turn_drift_2_to_12_m=max(norm_xy(r['position_world'],turn[0]['position_world']) for r in turn),
        legacy_turn_drift_2p5_to_11p9_m=result['turn_audit']['max_turn_drift_m'],
        brake_yaw_change_deg=angle_travel(brake,'yaw_deg'),
        brake_mean_speed_mps=mean(r['speed_mps'] for r in brake),
        final_speed_mps=result['final_speed_mps'],
        final_both_feet_contact=all(f>=20 for f in rows[-1]['foot_force_n']),
        max_tilt_deg=max(r['tilt_deg'] for r in rows),failure=result['failure'],
        in_place_turn_pass=result['turn_audit']['pass'],
        video_qa=dict(frames=n,reported=reported,fps=fps,resolutions=list(dim),dark_frames=dark,
            passed=n==reported and dim=={(1280,720)} and fps==25 and not dark),
        physics_clock_error_s=result['turn_audit']['max_physics_clock_error_s'])
    (file.parent/'footstep_validated.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    reports[file.parent.name]={k:v for k,v in report.items() if k!='foot_events'}
(args.root/'official_foot_summary.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
print(json.dumps(reports,indent=2))
