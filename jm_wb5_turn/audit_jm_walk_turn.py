"""Decode every video frame and independently summarize recorded motion phases."""
import argparse
import json
import math
from pathlib import Path
from statistics import mean
import sys
import cv2
from PIL import Image,ImageDraw

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from jm_wb5_turn.footstep_metrics import foot_events,angle_travel

p=argparse.ArgumentParser()
p.add_argument('run',type=Path)
args=p.parse_args()
rows=json.loads((args.run/'telemetry.json').read_text())
result=json.loads((args.run/'result.json').read_text())
plan=json.loads((args.run/'plan.json').read_text())
runtime=json.loads((args.run/'runtime_contract.json').read_text())
reports={}
for phase in plan['phases']:
    rr=[r for r in rows if r['phase']==phase]
    if not rr: continue
    counts=[sum(e['verified_lift_landing'] for e in foot_events(rows,k,rr[0]['time_s'],rr[-1]['time_s'])) for k in range(2)]
    reports[phase]=dict(start_s=rr[0]['time_s'],end_s=rr[-1]['time_s'],
        displacement_xy_m=math.dist(rr[0]['position_world'][:2],rr[-1]['position_world'][:2]),
        pelvis_turn_deg=angle_travel(rr,'yaw_deg'),torso_turn_deg=angle_travel(rr,'torso_yaw_world_deg'),
        foot_turn_deg=[angle_travel(rr,'foot_yaw_deg',k) for k in range(2)],
        verified_foot_lift_land=counts,mean_body_speed_mps=mean(r['speed_mps'] for r in rr),
        final_goal_error_m=rr[-1]['goal_distance_m'],final_heading_error_deg=rr[-1]['heading_error_deg'])
cap=cv2.VideoCapture(str(args.run/'jm_walk_left_walk_720p.mp4'))
fps=cap.get(cv2.CAP_PROP_FPS); reported=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
sample_indices=sorted(set(round((reported-1)*i/11) for i in range(12)))
n=0; dark=[]; resolutions=set(); tiles=[]
while True:
    ok,frame=cap.read()
    if not ok: break
    resolutions.add((frame.shape[1],frame.shape[0]))
    if frame[110:650].mean()<4: dark.append(n)
    if n in sample_indices:
        im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
        im.resize((640,360)).save(args.run/f'review_frame_{n:04d}.jpg')
        thumb=im.resize((426,240))
        tile=Image.new('RGB',(426,262),(20,26,34)); tile.paste(thumb,(0,22))
        ImageDraw.Draw(tile).text((8,4),f'frame {n} / t~{n/fps:.2f}s',fill='white')
        tiles.append(tile)
    n+=1
cap.release()
sheet=Image.new('RGB',(1278,262*math.ceil(len(tiles)/3)),(20,26,34))
for i,tile in enumerate(tiles): sheet.paste(tile,((i%3)*426,(i//3)*262))
sheet.save(args.run/'motion_contact_sheet.jpg')
qa=dict(decoded=n,reported=reported,fps=fps,duration_s=n/fps,resolutions=list(resolutions),dark_frames=dark,
        passed=n==reported==result['frames'] and fps==25 and resolutions=={(1280,720)} and not dark)
audit=dict(completed=result['completed'],failure=result['failure'],phases=reports,video=qa,
    max_clock_error_s=max(abs(r['physics_clock_error_s']) for r in runtime),all_pd_checks_pass=all(r['pd_ok'] for r in runtime),
    final_position=rows[-1]['position_world'],final_goal_error_m=result['final_distance_m'],
    final_heading_error_deg=rows[-1]['heading_error_deg'],turn_drift_m=result['turn_max_drift_m'],
    in_place_20cm_pass=result['in_place_20cm_pass'],max_environment_force_proxy_n=result['max_environment_contact_impulse_over_dt_n'],
    max_tilt_deg=result['max_tilt_deg'],source_unchanged=all(result['source_unchanged'].values()),
    feet_still_stop_claimed=False,scope='Official native H1 in JM; not modified gripper H1 compatibility')
(args.run/'validated_summary.json').write_text(json.dumps(audit,indent=2))
print(json.dumps(audit,indent=2))
