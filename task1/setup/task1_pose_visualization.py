#!/usr/bin/env python3
"""Render pose comparisons independently of model inference."""
import numpy as np
from PIL import Image, ImageDraw

GT_COLOR = (255, 65, 210)
POSE_COLOR = (0, 225, 255)
EDGES = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]


def render_pose_overlay(rgb, K, corners, gt, pose, translation_error, rotation_error, elapsed):
    im = Image.fromarray(rgb)
    draw = ImageDraw.Draw(im)
    for tf, color, dashed in [(gt, GT_COLOR, False), (pose, POSE_COLOR, True)]:
        xyz = corners @ tf[:3,:3].T + tf[:3,3]
        uvw = xyz @ K.T
        uv = uvw[:,:2] / uvw[:,2:]
        for a, b in EDGES:
            if min(xyz[a,2], xyz[b,2]) <= 0:
                continue
            start, end = uv[a], uv[b]
            if dashed:
                length = float(np.linalg.norm(end-start))
                if length == 0:
                    continue
                for offset in np.arange(0, length, 14):
                    points = [tuple(start+(end-start)*offset/length),
                              tuple(start+(end-start)*min(offset+8, length)/length)]
                    draw.line(points, fill=(20,20,30), width=4)
                    draw.line(points, fill=color, width=2)
            else:
                points = [tuple(start), tuple(end)]
                draw.line(points, fill=(20,20,30), width=6)
                draw.line(points, fill=color, width=4)
    draw.rectangle((5,5,440,44), fill=(18,23,32))
    draw.text((10,10), 'GT: magenta solid', fill=GT_COLOR)
    draw.text((160,10), 'FoundationPose: cyan dashed', fill=POSE_COLOR)
    draw.text((10,27), '%.1f mm | %.1f deg (box symmetry) | %.2f s' %
              (translation_error*1000, rotation_error, elapsed), fill='white')
    return im


if __name__ == '__main__':
    import argparse
    import hashlib
    import json
    from pathlib import Path
    import shutil

    parser = argparse.ArgumentParser(description='Re-render saved poses; does not run inference.')
    parser.add_argument('--observations', required=True, type=Path)
    parser.add_argument('--results', required=True, type=Path)
    args = parser.parse_args()
    report_path = args.results/'report.json'
    report = json.loads(report_path.read_text())
    capture = json.loads((args.observations/'capture_report.json').read_text())
    K = np.loadtxt(args.observations/'K.txt')
    corners = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                        [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]]) * np.array(capture['extents_m']) / 2
    preserved = [report_path] + list(args.results.glob('*_T_C_O.txt'))
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in preserved}
    backup = args.results/'overlays_before_color_update'
    backup.mkdir(exist_ok=True)
    for item in report['frames']:
        frame = args.observations/item['frame']
        output = args.results/(item['frame']+'_overlay.png')
        if output.exists() and not (backup/output.name).exists():
            shutil.copy2(output, backup/output.name)
        rgb = np.array(Image.open(frame/'rgb.png').convert('RGB'))
        gt = np.loadtxt(frame/'T_C_O_gt.txt')
        pose = np.loadtxt(args.results/(item['frame']+'_T_C_O.txt'))
        im = render_pose_overlay(rgb, K, corners, gt, pose, item['translation_error_m'],
                                 item['box_symmetry_rotation_error_deg'], item['inference_seconds'])
        im.save(output)
        print(output)
    assert hashes == {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in preserved}
    (args.results/'visualization_update.json').write_text(json.dumps({
        'gt': 'magenta solid', 'estimate': 'cyan dashed', 'inference_rerun': False,
        'preserved_result_sha256': hashes, 'original_overlays': str(backup),
    }, indent=2)+'\n')
