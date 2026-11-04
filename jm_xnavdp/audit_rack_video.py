"""Extract original rollout frames/crops for visual audit, without rerunning physics."""
import json
import subprocess
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
base=ROOT/'artifacts/xnavdp_20260930'
out=base/'rack_visual_audit_003'
out.mkdir(exist_ok=False)
summary={}
for name,times,roi in [
    ('jm_single_robot_001',[5,8,10,12,14,16.1,18.1,24],(570,135,230,300)),
    ('jm_transfer_004',[0,2,4,5,6,7,7.5,7.8],(420,135,230,300)),
]:
    run=base/name
    video=run/'jm_xnavdp_physical_720p.mp4'
    cap=cv2.VideoCapture(str(video)); fps=cap.get(cv2.CAP_PROP_FPS)
    count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    tiles=[]; samples=[]
    for t in times:
        i=min(int(round(t*fps)),count-1)
        cap.set(cv2.CAP_PROP_POS_FRAMES,i); ok,f=cap.read()
        if not ok: raise RuntimeError((name,i))
        cv2.imwrite(str(out/f'{name}_{i:04d}_original.jpg'),f)
        x,y,w,h=roi
        crop=f[y:y+h,x:x+w]
        tile=np.zeros((360,320,3),np.uint8)
        crop=cv2.resize(crop,(245,320),interpolation=cv2.INTER_NEAREST)
        tile[32:352,37:282]=crop
        cv2.putText(tile,f'frame {i} / {i/fps:.2f}s',(9,22),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        tiles.append(tile)
        samples.append({'frame':i,'time_s':i/fps,'source_roi_xywh':roi})
    sheet=np.vstack([np.hstack(tiles[:4]),np.hstack(tiles[4:])])
    cv2.imwrite(str(out/f'{name}_rack_sheet.jpg'),sheet)
    # Scene region only; old validator counted overlay/inset when checking black frames.
    cap.set(cv2.CAP_PROP_POS_FRAMES,0)
    dark=[]; i=0
    while True:
        ok,f=cap.read()
        if not ok: break
        if np.mean(f[90:490,:])<3: dark.append(i)
        i+=1
    cap.release()
    summary[name]={'samples':samples,'decoded_frames':i,'dark_main_scene_frames':dark,
                   'dark_main_scene_count':len(dark),'fps':fps}
    if name=='jm_single_robot_001':
        # Actual source footage only, with main scene and labelled crop, 8-24s.
        cap=cv2.VideoCapture(str(video))
        raw=out/'jm_rack_turn_zoom_preview.mp4'
        writer=cv2.VideoWriter(str(raw),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1280,720))
        cap.set(cv2.CAP_PROP_POS_FRAMES,int(8*fps))
        for i in range(int(8*fps),int(24*fps)):
            ok,f=cap.read()
            if not ok: break
            canvas=np.zeros((720,1280,3),np.uint8)
            canvas[115:520,:720]=cv2.resize(f,(720,405))
            x,y,w,h=roi
            crop=cv2.resize(f[y:y+h,x:x+w],(460,600),interpolation=cv2.INTER_NEAREST)
            canvas[70:670,790:1250]=crop
            cv2.putText(canvas,f'Original recording t={i/fps:.2f}s | rack proximity review',(20,32),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(canvas,'LEFT: full frame / RIGHT: enlarged crop (no new viewpoint)',(20,60),cv2.FONT_HERSHEY_SIMPLEX,.6,(210,210,210),1,cv2.LINE_AA)
            if i in dark:
                cv2.putText(canvas,'MAIN SCENE BLACK: cannot assess this frame',(20,555),cv2.FONT_HERSHEY_SIMPLEX,.65,(80,180,255),2,cv2.LINE_AA)
            writer.write(canvas)
        writer.release(); cap.release()
        subprocess.run(['C:/isaacsim/kit/python/Lib/site-packages/imageio_ffmpeg/binaries/ffmpeg-win-x86_64-v7.1.exe',
                        '-hide_banner','-loglevel','error','-i',str(raw),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p',
                        '-movflags','+faststart',str(out/'jm_rack_turn_zoom_720p.mp4')],check=True)
(out/'audit.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps({k:{x:y for x,y in v.items() if x not in ['samples','dark_main_scene_frames']} for k,v in summary.items()},indent=2))
