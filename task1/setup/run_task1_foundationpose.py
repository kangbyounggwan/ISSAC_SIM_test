#!/usr/bin/env python3
"""Model-based FoundationPose smoke experiment using saved, synchronized inputs."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

p = argparse.ArgumentParser()
p.add_argument('--observations', required=True, type=Path)
p.add_argument('--output', required=True, type=Path)
p.add_argument('--repo', required=True, type=Path)
args = p.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
sys.path.insert(0, str(args.repo.resolve()))
os.environ['PATH'] = str(Path(sys.executable).parent) + os.pathsep + os.environ.get('PATH', '')
os.environ.setdefault('MAX_JOBS', '8')
# These are the official downloaded checkpoints; the upstream loader predates torch 2.6.
os.environ['TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD'] = '1'
os.environ.setdefault('MPLBACKEND', 'Agg')
import numpy as np
import torch
import trimesh
from PIL import Image
from task1_pose_visualization import render_pose_overlay
from estimater import FoundationPose
from Utils import set_seed

set_seed(42)
K = np.loadtxt(args.observations / 'K.txt')
mesh = trimesh.load(args.observations / 'object.obj', force='mesh', process=False)
# This scene has a uniform displayColor, not an image texture. Avoid a texture material with no image.
mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=[28,112,51,255])
t0 = time.monotonic()
est = FoundationPose(model_pts=mesh.vertices, model_normals=mesh.vertex_normals,
                     mesh=mesh, debug=0, debug_dir=str(args.output/'debug'))
load_seconds = time.monotonic()-t0
results = []
symmetries = [np.diag(v) for v in ([1,1,1], [1,-1,-1], [-1,1,-1], [-1,-1,1])]
extents = mesh.extents
corners = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                    [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])*extents/2

for frame in sorted(args.observations.glob('frame_*')):
    rgb = np.array(Image.open(frame/'rgb.png').convert('RGB'))
    depth = np.load(frame/'depth_m.npy')
    mask = np.array(Image.open(frame/'mask.png')) > 0
    gt = np.loadtxt(frame/'T_C_O_gt.txt')
    t0 = time.monotonic()
    with torch.inference_mode():
        pose = est.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=5)
    torch.cuda.synchronize()
    elapsed = time.monotonic()-t0
    if not np.isfinite(pose).all() or not np.allclose(pose[3], [0,0,0,1], atol=1e-4):
        raise RuntimeError('Invalid pose output')
    np.savetxt(args.output/(frame.name+'_T_C_O.txt'), pose)
    R_error = [np.degrees(np.arccos(np.clip((np.trace(pose[:3,:3].T @ gt[:3,:3] @ s)-1)/2,-1,1))) for s in symmetries]
    translation_error = float(np.linalg.norm(pose[:3,3]-gt[:3,3]))
    im = render_pose_overlay(rgb, K, corners, gt, pose, translation_error, min(R_error), elapsed)
    im.save(args.output/(frame.name+'_overlay.png'))
    item = {'frame':frame.name,'T_C_O':pose.tolist(),'translation_error_m':translation_error,
            'rotation_error_deg':float(R_error[0]),'box_symmetry_rotation_error_deg':float(min(R_error)),
            'inference_seconds':elapsed,'initialization':'register independently; no GT pose supplied',
            'mask_source':'simulator ground truth'}
    results.append(item)
    print(json.dumps(item),flush=True)
    report = {'status':'success' if len(results)==len(list(args.observations.glob('frame_*'))) else 'running',
              'method':'FoundationPose model-based register', 'seed':42, 'load_seconds':load_seconds,
              'iterations':5, 'torch':torch.__version__, 'numpy':np.__version__, 'frames':results,
              'limitations':['GT masks','plain symmetric Cube proxy, not detailed PCB CAD','three static views; no benchmark or robot execution']}
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
