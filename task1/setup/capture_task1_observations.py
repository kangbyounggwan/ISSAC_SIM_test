#!/usr/bin/env python3
"""Capture synchronized static-scene model inputs without changing the source USD."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import traceback

parser = argparse.ArgumentParser()
parser.add_argument('--world', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--target', default='/World/Dadong/F2/PCBStacks/wet_recv/pcb7')
parser.add_argument('--gpu', type=int, default=1)
args, _ = parser.parse_known_args()
args.output.mkdir(parents=True, exist_ok=True)
if (args.output / 'capture_report.json').exists():
    raise SystemExit('Use a new output directory; existing captures are preserved.')
report = {'status': 'starting', 'target': args.target, 'world': str(args.world.resolve()), 'frames': []}
started = time.monotonic()

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'width': 640, 'height': 480,
                     'active_gpu': args.gpu, 'physics_gpu': args.gpu, 'multi_gpu': False, 'sync_loads': True})

try:
    import carb
    import numpy as np
    from PIL import Image, ImageDraw
    import omni.usd
    import omni.replicator.core as rep
    from pxr import Gf, Usd, UsdGeom
    from isaacsim.core.utils.semantics import add_update_semantics

    def dump(path, value):
        def convert(x):
            if isinstance(x, np.ndarray): return x.tolist()
            if isinstance(x, np.generic): return x.item()
            raise TypeError(type(x).__name__)
        path.write_text(json.dumps(value, indent=2, default=convert) + '\n')

    carb.settings.get_settings().set('/rtx/post/histogram/enabled', False)
    context = omni.usd.get_context()
    context.open_stage(str(args.world.resolve()))
    for _ in range(30): app.update()
    stage = context.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    if meters_per_unit != 1.0:
        raise RuntimeError('This adapter expects a meter-denominated stage.')
    target = stage.GetPrimAtPath(args.target)
    if not target or target.GetTypeName() != 'Cube':
        raise RuntimeError('Initial adapter requires the actual Cube PCB from this scene.')
    for prim in target.GetParent().GetChildren():
        if prim.GetTypeName() == 'Cube':
            add_update_semantics(prim, 'pcb' if prim.GetName().startswith('pcb') else 'rack')
    affine = np.array(UsdGeom.Xformable(target).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
    scales = np.linalg.norm(affine[:3, :3], axis=0)
    T_W_O = affine.copy()
    T_W_O[:3, :3] /= scales
    if not np.allclose(T_W_O[:3, :3].T @ T_W_O[:3, :3], np.eye(3), atol=1e-6):
        raise RuntimeError('Sheared object transform needs a different mesh conversion.')
    extents = scales * float(UsdGeom.Cube(target).GetSizeAttr().Get())
    corners = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                        [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]]) * extents / 2
    faces = [(1,3,2),(1,4,3),(5,6,7),(5,7,8),(1,2,6),(1,6,5),
             (4,8,7),(4,7,3),(1,5,8),(1,8,4),(2,3,7),(2,7,6)]
    obj = '# Exact source Cube geometry, scale baked in, meters; origin at object center.\n'
    obj += 'mtllib object.mtl\nusemtl pcb\n'
    obj += ''.join('v %.10f %.10f %.10f\n' % tuple(v) for v in corners)
    obj += ''.join('f %d %d %d\n' % f for f in faces)
    (args.output / 'object.obj').write_text(obj)
    (args.output / 'object.mtl').write_text('newmtl pcb\nKd 0.11 0.44 0.20\nKa 0.11 0.44 0.20\n')
    np.savetxt(args.output / 'T_W_O.txt', T_W_O)
    camera = UsdGeom.Camera.Define(stage, '/World/Task1CaptureCamera')
    camera.CreateFocalLengthAttr(24.0)
    camera.CreateHorizontalApertureAttr(24.0)
    camera.CreateVerticalApertureAttr(18.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000))
    camera_op = UsdGeom.Xformable(camera).AddTransformOp()
    K = np.array([[640., 0., 320.], [0., 640., 240.], [0., 0., 1.]])
    np.savetxt(args.output / 'K.txt', K)
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product(str(camera.GetPath()), (640, 480))
    ann = {name: rep.AnnotatorRegistry.get_annotator(name) for name in
           ['rgb', 'distance_to_image_plane', 'instance_segmentation', 'CameraParams']}
    for a in ann.values(): a.attach(product)
    center = T_W_O[:3, 3]
    camera_offsets = [(-0.8,-0.9,0.75),(-0.65,-0.75,1.05),(-0.8,0.9,0.75)]
    previews = []
    for idx, offset in enumerate(camera_offsets):
        frame = args.output / ('frame_%03d' % idx)
        frame.mkdir()
        eye = center + np.array(offset)
        gl_matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*center), Gf.Vec3d(0,0,1)).GetInverse()
        camera_op.Set(gl_matrix)
        for _ in range(20): app.update()
        rep.orchestrator.step(rt_subframes=32)
        rgb = np.asarray(ann['rgb'].get_data()).copy()[:, :, :3]
        depth = np.asarray(ann['distance_to_image_plane'].get_data()).copy().astype(np.float32)
        segmentation = ann['instance_segmentation'].get_data()
        labels = segmentation['info']['idToLabels']
        ids = [int(k) for k, v in labels.items() if str(v) == args.target]
        if len(ids) != 1:
            dump(frame / 'segmentation_info.json', segmentation['info'])
            raise RuntimeError('Target instance ID not unambiguously found: %r' % labels)
        mask = np.asarray(segmentation['data']) == ids[0]
        depth[~np.isfinite(depth) | (depth <= 0)] = 0
        valid = mask & (depth > 0)
        if valid.sum() < 300:
            raise RuntimeError('Target is not sufficiently visible in view %d: %d pixels' % (idx, valid.sum()))
        T_W_C = np.array(gl_matrix).T @ np.diag([1., -1., -1., 1.])
        T_C_O = np.linalg.inv(T_W_C) @ T_W_O
        yy, xx = np.nonzero(valid)
        z = depth[yy, xx]
        pc = np.stack([(xx-K[0,2])*z/K[0,0], (yy-K[1,2])*z/K[1,1], z], axis=1)
        pc_o = (pc - T_C_O[:3,3]) @ T_C_O[:3,:3]
        q = np.abs(pc_o) - extents / 2
        sdf = np.linalg.norm(np.maximum(q,0),axis=1) + np.minimum(np.max(q,axis=1),0)
        residual = np.abs(sdf)
        p99 = float(np.quantile(residual,0.99))
        # Independent geometry check detects wrong depth conventions, units, or matrix direction.
        if p99 > 0.006:
            raise RuntimeError('Depth vs source mesh mismatch: p99=%.6f m' % p99)
        Image.fromarray(rgb).save(frame / 'rgb.png')
        Image.fromarray((mask*255).astype(np.uint8)).save(frame / 'mask.png')
        np.save(frame / 'depth_m.npy', depth)
        np.save(frame / 'instance_ids.npy', segmentation['data'])
        np.save(frame / 'object_pc_camera.npy', pc.astype(np.float32))
        np.savetxt(frame / 'T_W_C.txt', T_W_C)
        np.savetxt(frame / 'T_C_O_gt.txt', T_C_O)
        dump(frame / 'camera_params_raw.json', ann['CameraParams'].get_data())
        dump(frame / 'segmentation_info.json', segmentation['info'])
        visual = rgb.copy()
        visual[mask] = (visual[mask]*0.5 + np.array([255,70,10])*0.5).astype(np.uint8)
        vis = Image.fromarray(visual)
        draw = ImageDraw.Draw(vis)
        projected = corners @ T_C_O[:3,:3].T + T_C_O[:3,3]
        uv = projected @ K.T
        uv = uv[:,:2] / uv[:,2:]
        edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
        for i,j in edges: draw.line([tuple(uv[i]),tuple(uv[j])],fill=(255,255,0),width=2)
        draw.text((12,12), 'GT mask + exact USD box | frame %03d' % idx, fill='white', stroke_width=1, stroke_fill='black')
        vis.save(frame / 'gt_overlay.png')
        previews.append(vis)
        info = {'id': frame.name, 'valid_target_pixels': int(valid.sum()), 'target_instance_id': ids[0],
                'depth_range_m': [float(z.min()),float(z.max())], 'depth_mesh_residual_p99_m': p99,
                'T_W_C': T_W_C, 'T_C_O_gt': T_C_O,
                'mask_source': 'Isaac Sim ground truth; not a segmentation model',
                'simulation': 'static scene; no walking, contacts or grasp execution'}
        dump(frame / 'frame.json', info)
        report['frames'].append(info)
        print('[FACTOR] captured', frame.name, int(valid.sum()), 'pixels; geometry p99', p99, flush=True)
    for a in ann.values(): a.detach(product)
    product.destroy()
    preview = Image.new('RGB',(640*len(previews),480))
    for idx, im in enumerate(previews): preview.paste(im,(idx*640,0))
    preview.save(args.output / 'observations_preview.jpg')
    report.update(status='success', extents_m=extents, T_W_O=T_W_O, K=K,
                  coordinate_contract='T_A_B maps column vectors from B into A; C is OpenCV (+X right,+Y down,+Z forward); O is centered mesh; meters',
                  source_sha256=hashlib.sha256(args.world.read_bytes()).hexdigest(),
                  elapsed_seconds=time.monotonic()-started)
    dump(args.output / 'capture_report.json', report)
except Exception:
    report.update(status='failed',error=traceback.format_exc())
    (args.output / 'capture_report.json').write_text(json.dumps(report,indent=2,default=lambda x:x.tolist())+'\n')
    raise
finally:
    app.close()
