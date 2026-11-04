"""Read complete run evidence; produce reproducible video QA and comparison."""
import json
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'artifacts/xnavdp_20260930'
OUT = BASE/'runtime_dock_review'


def read(run, name):
    return json.loads((BASE/run/name).read_text(encoding='utf-8'))


def video_qa(run):
    cap = cv2.VideoCapture(str(BASE/run/'jm_xnavdp_physical_720p.mp4'))
    fps = cap.get(cv2.CAP_PROP_FPS)
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    wanted = set(np.linspace(0, max(0,count-1), 8, dtype=int))
    tiles=[]; dark=[]; decoded=0; sizes=set()
    while True:
        ok, frame = cap.read()
        if not ok: break
        sizes.add(tuple(frame.shape[:2]))
        # Exclude top overlay and inset; never delete bad frames.
        if np.mean(frame[90:490, :940]) < 3: dark.append(decoded)
        if decoded in wanted:
            tile=cv2.resize(frame,(640,360))
            cv2.rectangle(tile,(0,0),(640,25),(0,0,0),-1)
            cv2.putText(tile,f'frame {decoded} / t={decoded/fps:.2f}s',(10,18),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            tiles.append(tile)
        decoded+=1
    cap.release()
    if len(tiles)==8:
        sheet=np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,8,2)])
        cv2.imwrite(str(OUT/(run+'_frames.jpg')),sheet)
    return dict(fps=fps,declared_frames=count,decoded_frames=decoded,sizes=list(sizes),black_main_frames=dark)


def main():
    OUT.mkdir(exist_ok=True)
    original='jm_policy_compare_wb5_recheck_neg_001'
    runtime='jm_runtime_contract_neg_001'
    dock='jm_dock_position_first_001'
    old=read(original,'telemetry.json'); new=read(runtime,'telemetry.json')
    assert len(old)==len(new)
    report=dict(runtime_result=read(runtime,'result.json'),
        baseline_max_position_delta_m=max(float(np.linalg.norm(np.array(a['position_world'])-b['position_world'])) for a,b in zip(old,new)),
        baseline_max_yaw_delta_deg=max(abs(a['yaw_deg']-b['yaw_deg']) for a,b in zip(old,new)),
        runtime_audit=read(runtime,'runtime_contract.json'), dock_result=read(dock,'result.json'))
    before=read('jm_single_robot_001','telemetry.json'); after=read(dock,'telemetry.json')
    report['at_fixed_times'] = []
    for t in [8,10,12,16,20,24]:
        row={'time_s':t}
        for label, data in [('before',before),('after',after)]:
            r=next((r for r in data if abs(r['time_s']-t)<1e-6),None)
            row[label]=None if r is None else {k:r[k] for k in ['goal_distance_m','yaw_error_deg','speed_mps','phase','velocity_command','dwell_s']}
        report['at_fixed_times'].append(row)
    report['video_qa']={run:video_qa(run) for run in [runtime,dock]}
    (OUT/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='runtime_audit'},indent=2))


if __name__=='__main__': main()
