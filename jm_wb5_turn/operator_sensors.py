"""Observation-only RGB-D + PhysX lidar rig for the local JM operator console.

No policy inputs, collision settings, goals, or robot poses are changed.
"""
import json
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT = 1280, 720
NEAR, FAR = .2, 20.


def depth_preview(depth):
    depth = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth >= .05) & (depth < FAR)
    normalized = np.clip(np.nan_to_num(depth, nan=FAR, posinf=FAR) / FAR, 0, 1)
    stops = np.array([[240, 100, 56], [242, 211, 91], [61, 197, 191], [52, 100, 186], [34, 42, 99]])
    channels = [np.interp(normalized, np.linspace(0, 1, len(stops)), stops[:, c]) for c in range(3)]
    rgb = np.stack(channels, axis=-1).astype(np.uint8)
    rgb[~valid] = 0
    return rgb, valid


def filter_lidar(points, ranges):
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    ranges = np.asarray(ranges, dtype=np.float32).reshape(-1)
    if len(points) != len(ranges):
        raise ValueError('Lidar point/range count mismatch')
    valid = np.isfinite(points).all(axis=1) & np.isfinite(ranges) & (ranges >= NEAR) & (ranges < FAR-.001)
    # The SDK supplies sensor-local XYZ. Verify the frame using measured range.
    if valid.any() and not np.allclose(np.linalg.norm(points[valid], axis=1), ranges[valid], atol=.03, rtol=.01):
        raise ValueError('Lidar XYZ is not consistent with sensor-local measured ranges')
    return points[valid], valid


def lidar_preview(points):
    img = Image.new('RGB', (WIDTH, HEIGHT), (8, 18, 28))
    draw = ImageDraw.Draw(img)
    cx, cy, scale = 640, 378, 14.
    for radius in (5, 10, 15, 20):
        r = radius*scale
        draw.ellipse((cx-r, cy-r, cx+r, cy+r), outline=(39, 63, 80), width=1)
        draw.text((cx+6, cy-r+4), f'{radius}m', fill=(115, 146, 163))
    draw.line((cx, 83, cx, 675), fill=(50, 80, 96))
    draw.line((cx-300, cy, cx+300, cy), fill=(50, 80, 96))
    for x, y, z in points:
        u, v = cx-float(y)*scale, cy-float(x)*scale
        color = (113, 221, 203) if z >= -.5 else (72, 130, 173)
        draw.ellipse((u-1.5, v-1.5, u+1.5, v+1.5), fill=color)
    draw.polygon([(cx, cy-11), (cx-7, cy+8), (cx, cy+4), (cx+7, cy+8)], fill=(255, 210, 95))
    draw.text((cx+12, 89), '+X / ROBOT FORWARD', fill=(170, 206, 219))
    draw.text((30, 690), 'TOP-DOWN XY PROJECTION | sensor-local XYZ | collider returns only | not RGB-derived', fill=(136, 170, 184))
    return img


class OperatorSensors:
    def __init__(self, stage, robot_path, out, bridge):
        import omni.kit.commands
        import omni.replicator.core as rep
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import Gf, UsdGeom
        enable_extension('isaacsim.sensors.physx')
        from isaacsim.sensors.physx import _range_sensor
        self.stage, self.out, self.bridge = stage, Path(out), bridge
        self.raw_dir = self.out/'sensor_raw'; self.raw_dir.mkdir()
        self.font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 19)
        self.mount = robot_path+'/torso_link'
        self.camera_path = self.mount+'/OperatorRGBD'
        self.lidar_path = self.mount+'/OperatorLidar'
        camera = UsdGeom.Camera.Define(stage, self.camera_path)
        camera.CreateFocalLengthAttr(18.)
        camera.CreateHorizontalApertureAttr(30.)
        camera.CreateVerticalApertureAttr(30.*9/16)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 200.))
        self.local_eye = (.22, 0., .62)
        matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*self.local_eye), Gf.Vec3d(1.22, 0., .52), Gf.Vec3d(0,0,1)).GetInverse()
        camera.AddTransformOp().Set(matrix)
        self.product = rep.create.render_product(self.camera_path, (WIDTH, HEIGHT))
        self.rgb = rep.AnnotatorRegistry.get_annotator('rgb'); self.rgb.attach([self.product])
        self.depth = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane'); self.depth.attach([self.product])
        ok, lidar = omni.kit.commands.execute('RangeSensorCreateLidar', path='/OperatorLidar', parent=self.mount,
            translation=Gf.Vec3d(.22,0.,.70), min_range=NEAR,max_range=FAR,
            draw_points=False,draw_lines=False,horizontal_fov=360.,vertical_fov=30.,
            horizontal_resolution=2.,vertical_resolution=3.,rotation_rate=0.,high_lod=True,
            yaw_offset=0.,enable_semantics=False)
        if not ok or not lidar: raise RuntimeError('PhysX lidar creation failed')
        self.lidar = _range_sensor.acquire_lidar_sensor_interface()
        self.writers = {}; self.count = 0; self.status = {}; self.samples = []; self.timings = []
        self.valid_counts = dict(rgb=0,depth=0,lidar=0)
        self.config = dict(source='JM Isaac Sim 5.1 simulation, not physical hardware',mount=self.mount,
            camera_path=self.camera_path,camera_local_eye=list(self.local_eye),camera_forward='+X, pitched down ~5.7 deg',
            intrinsic=[[768.,0.,640.],[0.,768.,360.],[0.,0.,1.]],resolution=[WIDTH,HEIGHT],
            depth_units='metres',depth_type='distance_to_image_plane',depth_display_range_m=[.05,FAR],
            lidar_path=self.lidar_path,lidar_type='PhysX SDK raycast lidar (not RTX optical model)',
            lidar_range_m=[NEAR,FAR],lidar_fov_deg=[360,30],lidar_resolution_deg=[2,3],
            lidar_frame='sensor local: +X forward, +Y left, +Z up',lidar_noise_model=False,
            video_fps=25,live_publish_every_frames=10,raw_save_every_frames=10,
            policy_uses_sensor_data=False,original_asset_write=False)
        (self.out/'sensor_manifest.json').write_text(json.dumps(self.config,indent=2),encoding='utf-8')

    def capture(self,t,publish=False):
        import imageio.v2 as imageio
        from pxr import UsdGeom
        start=time.perf_counter(); raw={}; images={}; statuses={}
        try:
            rgb=np.asarray(self.rgb.get_data())[:,:,:3].copy()
            if rgb.shape!=(HEIGHT,WIDTH,3): raise ValueError('RGB data unavailable')
            images['rgb']=Image.fromarray(rgb); raw['rgb']=rgb
            statuses['rgb']=dict(ready=True,source='torso-mounted RGB render',width=WIDTH,height=HEIGHT)
        except Exception as exc: statuses['rgb']=dict(ready=False,error=str(exc))
        try:
            depth=np.asarray(self.depth.get_data(),dtype=np.float32).reshape(HEIGHT,WIDTH).copy()
            color,valid=depth_preview(depth);images['depth']=Image.fromarray(color);raw['depth_m']=depth
            statuses['depth']=dict(ready=bool(valid.any()),source='distance_to_image_plane',units='m',
                valid_fraction=float(valid.mean()),min_m=float(depth[valid].min()) if valid.any() else None,max_m=float(depth[valid].max()) if valid.any() else None)
        except Exception as exc: statuses['depth']=dict(ready=False,error=str(exc))
        try:
            points=np.asarray(self.lidar.get_point_cloud_data(self.lidar_path)).copy()
            ranges=np.asarray(self.lidar.get_linear_depth_data(self.lidar_path)).copy()
            if ranges.size==0: raise ValueError('Lidar scan not ready')
            filtered,valid=filter_lidar(points,ranges);images['lidar']=lidar_preview(filtered)
            raw.update(lidar_points_local=points,lidar_ranges_m=ranges,lidar_valid=valid)
            statuses['lidar']=dict(ready=True,source='PhysX SDK raycast lidar',units='m',rays=int(ranges.size),hits=len(filtered),
                min_m=float(ranges.reshape(-1)[valid].min()) if valid.any() else None)
        except Exception as exc: statuses['lidar']=dict(ready=False,error=str(exc))
        for kind in ('rgb','depth','lidar'):
            info=statuses[kind];self.valid_counts[kind]+=int(info['ready'])
            image=images.get(kind,Image.new('RGB',(WIDTH,HEIGHT),(8,18,28)))
            draw=ImageDraw.Draw(image);draw.rectangle((0,0,WIDTH,63),fill=(8,18,28))
            title={'rgb':'ROBOT RGB | torso-mounted forward camera','depth':'DEPTH | image-plane metres | warm=near, blue=far, black=invalid','lidar':'LIDAR | PhysX raycast | 360 deg x 30 deg | top-down point cloud'}[kind]
            draw.text((15,8),title,font=self.font,fill='white')
            draw.text((15,34),f'JM SIMULATION | t={t:.2f}s | '+('MEASURED' if info['ready'] else 'UNAVAILABLE: '+info.get('error','No valid depth')),font=self.font,fill=(120,210,214) if info['ready'] else (255,125,110))
            if kind not in self.writers:self.writers[kind]=imageio.get_writer(str(self.out/f'sensor_{kind}_720p.mp4'),fps=25,codec='libx264',quality=7,macro_block_size=None)
            self.writers[kind].append_data(np.asarray(image))
            if publish:self.bridge.frame(image,kind)
        self.count+=1
        if publish:
            cache=UsdGeom.XformCache()
            raw['time_s']=np.array(t)
            raw['camera_local_to_world']=np.array(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(self.camera_path)))
            raw['lidar_local_to_world']=np.array(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(self.lidar_path)))
            file=f'sample_{len(self.samples):04d}.npz';np.savez_compressed(self.raw_dir/file,**raw)
            self.samples.append(dict(time_s=t,file=file,streams=statuses))
            self.status=dict(time_s=t,sequence=self.count,wall_time_unix=time.time(),streams=statuses)
            self.bridge.sensors(self.status)
        self.timings.append(time.perf_counter()-start)

    def preview(self, kind):
        """Selected live modality only; no MP4 encoding or raw dataset write."""
        if kind == 'rgb':
            rgb = np.asarray(self.rgb.get_data())[:, :, :3].copy()
            if rgb.shape != (HEIGHT, WIDTH, 3):
                raise ValueError('RGB data unavailable')
            return Image.fromarray(rgb)
        if kind == 'depth':
            depth = np.asarray(self.depth.get_data(), dtype=np.float32).reshape(HEIGHT, WIDTH)
            color, valid = depth_preview(depth)
            if not valid.any():
                raise ValueError('Depth data unavailable')
            return Image.fromarray(color)
        if kind == 'lidar':
            points = np.asarray(self.lidar.get_point_cloud_data(self.lidar_path)).copy()
            ranges = np.asarray(self.lidar.get_linear_depth_data(self.lidar_path)).copy()
            if not ranges.size:
                raise ValueError('Lidar scan not ready')
            points, _ = filter_lidar(points, ranges)
            return lidar_preview(points)
        raise ValueError('Unknown preview modality')

    def close(self):
        for writer in self.writers.values():writer.close()
        summary=dict(self.config,frames=self.count,valid_frames=self.valid_counts,samples=self.samples,
            capture_wall_s=float(sum(self.timings)),capture_p95_ms=float(np.percentile(self.timings,95)*1000) if self.timings else None)
        (self.out/'sensor_manifest.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
