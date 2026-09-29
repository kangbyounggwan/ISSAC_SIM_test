#!/usr/bin/env python3
"""Generate diffusion-only candidates on captured object PCs; no robot execution."""
import argparse
import json
import os
from pathlib import Path
import random
import time

p=argparse.ArgumentParser()
p.add_argument('--observations', required=True,type=Path)
p.add_argument('--output',required=True,type=Path)
p.add_argument('--models',required=True,type=Path)
args=p.parse_args()
args.output.mkdir(parents=True,exist_ok=False)
os.environ['GRASPGENX_CHECKPOINT_DIR']=str(args.models/'GraspGenXModel')
os.environ['GRASPGENX_GRIPPER_CFG_DIR']=str(args.models/'gripper_descriptions')
os.environ['TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD']='1'
os.environ.setdefault('MPLBACKEND','Agg')
import numpy as np
import torch
from PIL import Image,ImageDraw
from graspgenx.grasp_server import GraspGenXSampler
from graspgenx.utils.checkpoint_io import load_model_cfg

random.seed(42)
np.random.seed(42)
torch.manual_seed(42)
torch.cuda.manual_seed_all(42)
ckpt=args.models/'GraspGenXModel'/'release'
cfg=load_model_cfg(ckpt/'gen',ckpt/'dis')
t0=time.monotonic()
assets=args.models/'gripper_descriptions'/'gripper_descriptions'/'assets'
sampler=GraspGenXSampler(cfg,gripper_name='franka_panda',assets_dir=str(assets))
load_seconds=time.monotonic()-t0
K=np.loadtxt(args.observations/'K.txt')
results=[]
for frame in sorted(args.observations.glob('frame_*')):
    pc=np.load(frame/'object_pc_camera.npy')
    rng=np.random.default_rng(42)
    if len(pc)>8192: pc=pc[rng.choice(len(pc),8192,replace=False)]
    t0=time.monotonic()
    with torch.inference_mode():
        grasps,scores=GraspGenXSampler.run_inference(pc,sampler,grasp_threshold=-1,
            num_grasps=200,topk_num_grasps=40,min_grasps=40,max_tries=1,remove_outliers=False)
    torch.cuda.synchronize()
    elapsed=time.monotonic()-t0
    grasps=grasps.detach().cpu().numpy()
    scores=scores.detach().cpu().numpy()
    if len(grasps)==0 or not np.isfinite(grasps).all() or not np.isfinite(scores).all():
        raise RuntimeError('No finite candidate output')
    R=grasps[:,:3,:3]
    if not np.allclose(R.transpose(0,2,1)@R,np.eye(3),atol=1e-3):
        raise RuntimeError('Grasp rotations are not orthonormal')
    T_W_C=np.loadtxt(frame/'T_W_C.txt')
    world_grasps=T_W_C[None]@grasps
    np.savez(args.output/(frame.name+'_grasps.npz'),T_C_G=grasps,T_W_G=world_grasps,scores=scores)
    if not results:
        mesh=sampler.gripper.collision_mesh
        geometry={'source':frame.name+'_grasps.npz','gripper':'franka_panda','inference_only':True,
                  'vertices':mesh.vertices.tolist(),'faces':mesh.faces.tolist(),
                  'T_W_G':world_grasps[:5].tolist(),'scores':scores[:5].tolist()}
        (args.output/'preview_geometry.json').write_text(json.dumps(geometry)+'\n')
    im=Image.open(frame/'rgb.png').convert('RGB')
    draw=ImageDraw.Draw(im)
    # Schematic axes only: +Z approach, +X closing. Not a collision mesh or H1 hand.
    for rank,g in enumerate(grasps[:5]):
        origin=g[:3,3]
        xyz=np.vstack([origin,origin+g[:3,2]*0.08,origin-g[:3,0]*0.04,origin+g[:3,0]*0.04])
        if (xyz[:,2]<=0).any(): continue
        uvw=xyz@K.T
        uv=uvw[:,:2]/uvw[:,2:]
        color=(255,215-int(rank*25),0)
        draw.line([tuple(uv[0]),tuple(uv[1])],fill=(40,160,255),width=3)
        draw.line([tuple(uv[2]),tuple(uv[3])],fill=color,width=3)
        draw.text(tuple(uv[0]),str(rank+1),fill='white',stroke_width=1,stroke_fill='black')
    draw.text((10,10),'GraspGen-X | provisional Franka gripper | top 5 axes',fill='white',stroke_width=1,stroke_fill='black')
    draw.text((10,27),'Candidates only: no IK, collision test or physical grasp',fill='white',stroke_width=1,stroke_fill='black')
    im.save(args.output/(frame.name+'_overlay.png'))
    item={'frame':frame.name,'num_input_points':len(pc),'num_candidates':len(grasps),
          'score_min':float(scores.min()),'score_max':float(scores.max()),
          'scores_above_0_7':int((scores>=0.7).sum()),'inference_seconds':elapsed}
    results.append(item)
    print(json.dumps(item),flush=True)
    report={'status':'success' if len(results)==len(list(args.observations.glob('frame_*'))) else 'running',
            'method':'GraspGen-X','planner':'diffusion','seed':42,'num_sampled':200,'topk':40,
            'grasp_threshold':-1,'gripper':'franka_panda','gripper_is_final_h1_choice':False,
            'load_seconds':load_seconds,'torch':torch.__version__,'numpy':np.__version__,
            'frames':results,'coordinate_contract':'T_C_G maps model gripper frame into OpenCV camera; T_W_G=T_W_C@T_C_G',
            'limitations':['simulator GT instance mask','top-k returned even if scores are low','no scene collision, arm IK, balance or contact validation','scores are model scores, not measured success probabilities']}
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
