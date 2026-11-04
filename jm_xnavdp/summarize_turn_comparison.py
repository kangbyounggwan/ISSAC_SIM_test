"""Summarize measured comparisons and decode every video frame for QA."""
import json
from pathlib import Path
import subprocess
import os
os.environ.setdefault('MPLCONFIGDIR',str(Path(__file__).resolve().parents[1]/'artifacts/xnavdp_20260929/mpl_cache'))
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'artifacts/xnavdp_20260930'
OUT = BASE/'turn_comparison_review'
OUT.mkdir(exist_ok=True)
trials = []
for folder in sorted(BASE.glob('turn_compare_*')):
    if not folder.is_dir() or not (folder/'result.json').exists(): continue
    result = json.loads((folder/'result.json').read_text())
    manifest = json.loads((folder/'manifest.json').read_text())
    if manifest.get('drive_authoring') != 'stage_before_physics_initialization':
        # The first comparison harness lost runtime-only gains after the first
        # physics update. Preserve raw runs, but never rank policies from them.
        continue
    rows = json.loads((folder/'telemetry.json').read_text())
    cap = cv2.VideoCapture(str(folder/'turn_720p.mp4'))
    fps = cap.get(cv2.CAP_PROP_FPS)
    dark, shape, count = [], None, 0
    while True:
        ok, frame = cap.read()
        if not ok: break
        shape = frame.shape[:2]
        if float(frame[100:650,100:1180].mean())<12: dark.append(count)
        count += 1
    cap.release()
    audit = result['turn_audit']
    trials.append(dict(folder=folder.name, arguments=manifest['arguments'],
        yaw_deg=audit.get('accumulated_yaw_deg'), rate=audit.get('signed_mean_yaw_rate_rps'), late_rate=audit.get('signed_late_yaw_rate_rps'),
        drift_m=audit.get('max_turn_drift_m'), startup_drift_m=result['start_drift_m'], final_speed=result['final_speed_mps'],
        pass_turn=audit['pass'], completed=result['completed'], max_tilt=audit.get('max_tilt_deg'), failure=result['failure'],
        video=dict(frames=count,fps=fps,shape=shape,dark_frames=dark,decode_matches=count==result['frames']),
        policy_sha256=manifest['policy_sha256']))
    # One image per run showing 0 / 4 / 8 / 12 seconds (or final if aborted).
    cap = cv2.VideoCapture(str(folder/'turn_720p.mp4'))
    tiles = []
    for idx in [0,96,192,288]:
        idx = min(idx, count-1)
        cap.set(cv2.CAP_PROP_POS_FRAMES,idx)
        ok, frame = cap.read()
        if ok: tiles.append(cv2.resize(frame,(640,360)))
    cap.release()
    if len(tiles)==4:
        cv2.imwrite(str(OUT/(folder.name+'.jpg')),np.vstack([np.hstack(tiles[:2]),np.hstack(tiles[2:])]))

for folder_name,policy,sign in [
    ('jm_policy_compare_wb5_recheck_neg_001','wb5',-1),
    ('jm_turn_positive_001','wb5',1),
    ('jm_policy_compare_official_neg_001','official',-1),
    ('jm_policy_compare_official_pos_001','official',1),
]:
    folder=BASE/folder_name
    if not (folder/'result.json').exists(): continue
    result=json.loads((folder/'result.json').read_text())
    manifest=json.loads((folder/'manifest.json').read_text())
    rows=json.loads((folder/'telemetry.json').read_text())
    audit=result['turn_audit']
    cap=cv2.VideoCapture(str(folder/'jm_xnavdp_physical_720p.mp4'))
    count=0
    dark=[]
    fps=cap.get(cv2.CAP_PROP_FPS)
    shape=None
    while True:
        ok,frame=cap.read()
        if not ok: break
        shape=frame.shape[:2]
        if frame[100:490,120:1160].mean()<12: dark.append(count)
        count+=1
    cap.release()
    startup=[r for r in rows if r['time_s']<=2.]
    trials.append(dict(folder=folder_name,arguments=dict(policy=policy,asset='jm',contract='jm_deployed',sign=sign),
        yaw_deg=audit.get('accumulated_yaw_deg'),rate=audit.get('signed_mean_yaw_rate_rps'),late_rate=audit.get('signed_late_yaw_rate_rps'),
        drift_m=audit.get('max_turn_drift_m'),startup_drift_m=float(np.linalg.norm(np.array(startup[-1]['position_world'])[:2]-np.array(startup[0]['position_world'])[:2])),
        final_speed=rows[-1]['speed_mps'],pass_turn=audit['pass'],completed=result.get('diagnostic_completed',False),max_tilt=audit.get('max_tilt_deg'),
        failure=result['failure'],video=dict(frames=count,fps=fps,shape=shape,dark_frames=dark,decode_matches=count==result['frames']),
        policy_sha256=manifest['walker_sha256']))

(OUT/'summary.json').write_text(json.dumps(trials,indent=2))
original = BASE/'turn_compare_official_wb5contract_neg_001/asset_inventory.json'
modified = BASE/'turn_compare_official_jm_neg_001/asset_inventory.json'
if original.exists() and modified.exists():
    def inventory(path):
        records=json.loads(path.read_text())
        return {r['path'].rsplit('/',1)[-1].removesuffix('_joint'):r for r in records}
    a,b=inventory(original),inventory(modified)
    differences=[]
    ignore={'physics:velocity','physics:angularVelocity'}
    for name in sorted(a.keys() & b.keys()):
        attrs={}
        for key in sorted(a[name]['physics'].keys() | b[name]['physics'].keys()):
            if key in ignore or key.startswith('drive:'): continue
            av=a[name]['physics'].get(key)
            bv=b[name]['physics'].get(key)
            if av!=bv: attrs[key]={'original':av,'jm':bv}
        if attrs: differences.append({'canonical_name':name,'attributes':attrs})
    total=lambda path: sum(float(r['physics'].get('physics:mass',0)) for r in json.loads(path.read_text()))
    (OUT/'asset_differences.json').write_text(json.dumps(dict(original_authored_mass_sum=total(original),jm_authored_mass_sum=total(modified),
        added_names=sorted(b.keys()-a.keys()),differences=differences,
        caveat='Authored mass sums, not measured effective masses. All paths counted for mass; leaf-name matching used only for common body/joint differences. Colliders require separate audit.'),indent=2))
fig, ax = plt.subplots(2,2,figsize=(12,7),sharex=True,constrained_layout=True)
for trial in trials:
    a=trial['arguments']
    if a['contract']!='jm_deployed': continue
    rows=json.loads((BASE/trial['folder']/'telemetry.json').read_text())
    time=np.array([r['time_s'] for r in rows])
    yaw=np.degrees(np.unwrap(np.radians([r['yaw_deg'] for r in rows])))
    xy=np.array([r['position_world'][:2] for r in rows])
    col=0 if a['sign']==-1 else 1
    label=f"{a['policy'].upper()} / {a['asset']} asset"
    style='-' if a['asset']=='jm' else '--'
    color='#d85b42' if a['policy']=='wb5' else '#1979b9'
    ax[0,col].plot(time,yaw-yaw[0],style,label=label,color=color)
    ax[1,col].plot(time,np.linalg.norm(xy-xy[0],axis=1),style,label=label,color=color)
for col,title in enumerate(['Right turn: command -0.30 rad/s','Left turn: command +0.30 rad/s']):
    ax[0,col].set_title(title)
    for row in range(2):
        ax[row,col].axvspan(2,12,color='gray',alpha=.1)
        ax[row,col].grid(alpha=.25)
        ax[row,col].spines[['top','right']].set_visible(False)
    ax[1,col].set_xlabel('Simulation time (s)')
    ax[1,col].annotate('Official: FALL at 1.205s\n(before turn command)',xy=(1.2,.684),xytext=(3.,.70),
                       arrowprops=dict(arrowstyle='->',color='#1979b9'),fontsize=9,color='#1979b9')
ax[0,0].set_ylabel('Yaw change from initial pose (deg)')
ax[1,0].set_ylabel('XY displacement from initial pose (m)')
ax[0,0].legend(fontsize=8)
fig.suptitle('H1 turn response: same JM world, spawn and deployment settings\nOnly policy weights changed; shading = turn command (2-12s); curves include initial settling',fontsize=12)
fig.savefig(OUT/'turn_response.png',dpi=160)
plt.close(fig)

# Only generate the JM comparison when all four videos are complete, no reuse
# of terminal frames to disguise an aborted run.
chosen=[]
for policy,sign in [('wb5',-1),('official',-1),('wb5',1),('official',1)]:
    matches=[t for t in trials if t['arguments']['policy']==policy and t['arguments']['contract']=='jm_deployed' and t['arguments']['sign']==sign and t['completed']]
    if matches: chosen.append(matches[-1])
if len(chosen)==4 and len({t['video']['frames'] for t in chosen})==1:
    ffmpeg=Path('C:/isaacsim/kit/python/Lib/site-packages/imageio_ffmpeg/binaries/ffmpeg-win-x86_64-v7.1.exe')
    command=[str(ffmpeg),'-y','-v','error']
    for t in chosen: command += ['-i',str(BASE/t['folder']/'jm_xnavdp_physical_720p.mp4')]
    filters=';'.join(f'[{i}:v]scale=640:360[v{i}]' for i in range(4))
    filters += ';[v0][v1][v2][v3]xstack=inputs=4:layout=0_0|640_0|0_360|640_360[v]'
    command += ['-filter_complex',filters,'-map','[v]','-c:v','libx264','-crf','18','-pix_fmt','yuv420p',str(OUT/'jm_policy_comparison_720p.mp4')]
    subprocess.run(command,check=True)
print(json.dumps(trials,indent=2))
