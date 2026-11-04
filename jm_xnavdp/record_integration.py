"""720p in-world inspection video of recorded X-NavDP outputs, not locomotion.

Only the inspection camera and nonphysical diagnostic lines change. Physics is
paused; the H1 is NEVER animated along a predicted trajectory.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--world', choices=['jm', 'keti'], required=True)
p.add_argument('--out', type=Path, required=True)
p.add_argument('--seconds', type=int, default=24)
p.add_argument('--fps', type=int, default=24)
args = p.parse_args()
if args.seconds < 12 or args.fps != 24:
    raise ValueError('Record at least 12 seconds at 24fps')
if args.out.exists():
    raise RuntimeError('Use a new output directory')
args.out.mkdir(parents=True)
run = ROOT / 'artifacts/xnavdp_20260929'
capture = run / ('jm_capture_002' if args.world == 'jm' else 'keti_capture_001')
inference = run / (args.world + '_inference_001')
meta = json.loads((capture/'observation.json').read_text())
record = json.loads((inference/'result.json').read_text())
prediction = record['results'][-1]
audit = next(row for row in json.loads((run/'output_audit_001.json').read_text())['results'] if row['world'] == args.world)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


sources = dict(meta['source_sha256'])
for path in [capture/'observation.json', capture/'observation.npz', inference/'result.json', run/'output_audit_001.json', Path(__file__)]:
    sources[str(path.resolve())] = sha(path)
if not all(sha(path) == expected for path, expected in sources.items()):
    raise RuntimeError('Input world differs from captured model observation')
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'width': 1280, 'height': 720})
writer = None
try:
    import numpy as np
    import cv2
    import imageio.v2 as imageio
    import carb.settings
    import omni.usd
    import omni.timeline
    import omni.replicator.core as rep
    from PIL import Image, ImageDraw, ImageFont
    from pxr import Usd, UsdGeom, Gf
    omni.timeline.get_timeline_interface().stop()
    ctx = omni.usd.get_context()
    ctx.open_stage(meta['world'])
    for _ in range(20): app.update()
    stage = ctx.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    base = np.array(meta['base_world'])
    yaw = meta['yaw_rad']
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.])
    floor = 6.075 if args.world == 'jm' else 0.
    prefix = '/World/XNavDPVideo'
    if args.world == 'keti':
        manifest = json.loads((ROOT/'artifacts/keti_navigation_20260929/video_seed42_004/manifest.json').read_text())
        obs = manifest['scenario']['random_obstacle']
        cube = UsdGeom.Cube.Define(stage, prefix+'/RandomBox')
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*obs['center']))
        cube.AddScaleOp().Set(Gf.Vec3f(*obs['size']))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.94, .38, .07)])
        # Inspection-only robot at the existing trial's start; not a physics run.
        asset = ROOT/'jm_wb7_pinkik/assets/h1_wb7_standalone.usd'
        sources[str(asset)] = sha(asset)
        robot = UsdGeom.Xform.Define(stage, prefix+'/H1_DisplayOnly')
        robot.GetPrim().GetReferences().AddReference(asset.as_posix())
        robot.ClearXformOpOrder()
        robot.AddTranslateOp().Set(Gf.Vec3d(*base))
        robot.AddRotateZOp().Set(math.degrees(yaw))
        robot.GetPrim().SetCustomDataByKey('status', 'PAUSED DISPLAY ONLY - NO GAIT EXECUTION')
        robot_path = str(robot.GetPath())
    else:
        robot_path = '/World/H1Fleet/H1_1'
    robot_matrix_before = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def curve(name, points, color, width):
        c = UsdGeom.BasisCurves.Define(stage, prefix+'/'+name)
        c.CreateTypeAttr('linear')
        c.CreateCurveVertexCountsAttr([len(points)])
        c.CreatePointsAttr([Gf.Vec3f(float(x), float(y), float(z)) for x,y,z in points])
        c.CreateWidthsAttr([width]); c.SetWidthsInterpolation('constant')
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        return c

    zline = floor+.045
    selected = np.vstack([base[:2], np.asarray(prediction['world_path_xy'])])
    goal = np.array(meta['goal_world_xy'])
    # Diagnostic marker, NOT a physical support/collider.
    circle = [(*tuple(goal+.12*np.array([math.cos(t),math.sin(t)])), zline) for t in np.linspace(0,2*math.pi,65)]
    curve('Goal', circle, (.05,.95,.4), .024)
    # Grey candidates and blue selected path are model outputs, not hand-authored waypoints.
    from jm_xnavdp.adapter import world_path
    for j, candidate in enumerate(prediction['candidates_local_m']):
        xy = np.vstack([base[:2], world_path(candidate, base[:2], yaw)])
        curve('Candidate'+str(j), [(*v, zline-.009) for v in xy], (.5,.65,.7), .009)
    line = curve('Selected', [(*selected[0],zline), (*selected[0],zline)], (.03,.45,1.), .03)
    # Fixed start and goal are retained for the full video.
    camera = UsdGeom.Camera.Define(stage, prefix+'/InspectionCamera')
    camera.CreateFocalLengthAttr(22.)
    camera.CreateHorizontalApertureAttr(30.)
    camera.CreateVerticalApertureAttr(30.*9/16)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.05,200.))
    camera_op = camera.AddTransformOp()
    def camera_pose(t):
        if args.world == 'jm':
            center = np.array([*base[:2], floor+.8])+.55*forward
            eye = center+(2.65+.25*math.sin(t*.10))*right-2.5*forward+np.array([0.,0.,1.6])
        else:
            center = np.array([*base[:2],floor+.65])+2.2*forward
            eye = center+(5.3+.35*math.sin(t*.10))*right-2.4*forward+np.array([0.,0.,3.0])
        return Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*center), Gf.Vec3d(0,0,1)).GetInverse()
    camera_op.Set(camera_pose(0))
    carb.settings.get_settings().set('/rtx/post/histogram/enabled',False)
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product(str(camera.GetPath()),(1280,720))
    ann = rep.AnnotatorRegistry.get_annotator('rgb'); ann.attach([product])
    for _ in range(20): app.update()
    rep.orchestrator.step(rt_subframes=16,delta_time=0.,pause_timeline=True)
    output_path = args.out/(args.world+'_xnavdp_inspection_720p.mp4')
    writer = imageio.get_writer(str(output_path),fps=args.fps,codec='libx264',quality=8,macro_block_size=None)
    font = ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',20)
    small = ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',17)
    big = ImageFont.truetype('C:/Windows/Fonts/malgunbd.ttf',25)
    rgb_input = Image.open(capture/'rgb_720p.png').convert('RGB').resize((320,180))
    depth_input = Image.open(capture/'depth_720p.png').convert('RGB').resize((320,180))
    captions = (
        ['JM / 랙 앞 정지 목표', '관측: H1 초기 위치의 RGB-D', '파란 선: X-NavDP가 제안한 경로', '목표 초과 23.8cm · 간섭 proxy 검사 실패']
        if args.world == 'jm' else
        ['KETI / 출발 전 무작위 박스', '관측: 기존 seed42 출발 위치의 RGB-D', '파란 선: X-NavDP가 제안한 경로', '초기 4.8cm 되짚음 · 전체 우회 미검증']
    )
    count = args.seconds*args.fps
    for i in range(count):
        t = i/args.fps
        camera_op.Set(camera_pose(t))
        # Reveal recorded prediction nodes over presentation time, not simulation time.
        n = max(1,min(len(selected),1+int(max(0.,t-6)/8*(len(selected)-1))))
        pts = selected[:n] if n >= 2 else selected[:1].repeat(2,axis=0)
        line.GetCurveVertexCountsAttr().Set([len(pts)])
        line.GetPointsAttr().Set([Gf.Vec3f(float(v[0]),float(v[1]),zline) for v in pts])
        rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=True)
        raw=np.asarray(ann.get_data())
        if raw.ndim!=3 or raw.shape[:2]!=(720,1280): raise RuntimeError('Invalid 720p frame')
        img=Image.fromarray(raw[:,:,:3].copy()); draw=ImageDraw.Draw(img)
        draw.rectangle((0,0,1280,88),fill=(10,20,31))
        draw.text((20,8),captions[0]+'  |  X-NavDP 적용 검토',font=big,fill='white')
        draw.text((20,48),'정지 월드의 모델 경로 시각화 · 로봇 주행 아님 · 카메라만 이동',font=font,fill=(255,206,107))
        draw.rounded_rectangle((938,104,1274,565),radius=10,fill=(10,20,31))
        img.paste(rgb_input,(946,137)); img.paste(depth_input,(946,353))
        draw.text((948,108),'모델 입력 RGB (저장 관측)',font=small,fill='white')
        draw.text((948,324),'모델 입력 깊이 (0.1~5m)',font=small,fill='white')
        draw.text((948,537),f'측정 추론 {prediction["inference_wall_s"]:.3f}초',font=small,fill=(113,200,255))
        draw.rectangle((0,622,1280,720),fill=(10,20,31))
        phase=1 if t<6 else (2 if t<16 else 3)
        draw.text((20,631),f'{phase}/3  '+captions[phase],font=font,fill='white')
        detail=('녹색 원: 정지 목표 / 회색: 경로 후보 / 파랑: 선택 경로'
                if phase<3 else '실행 차단 유지 · 목표/허용오차 변경 없음 · 실제 이동 명령 0회')
        draw.text((20,665),detail,font=small,fill=(172,200,220))
        draw.text((1090,675),f'{t:04.1f} / {args.seconds}s',font=small,fill='white')
        writer.append_data(np.asarray(img))
        if i%96==0 or i==count-1: img.save(args.out/f'frame_{i:04d}.png')
        if i%96==0: print('VIDEO_PROGRESS',args.world,i,count,flush=True)
    writer.close(); writer=None
    after = UsdGeom.Xformable(stage.GetPrimAtPath(robot_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    assert np.allclose(np.array(robot_matrix_before),np.array(after),atol=0,rtol=0)
    assert all(sha(path)==expected for path,expected in sources.items())
    result={'status':'RECORDED_DIAGNOSTIC_NOT_ROBOT_MOTION','world':args.world,'video':str(output_path.resolve()),
            'resolution':[1280,720],'fps':args.fps,'frames':count,'duration_s':args.seconds,
            'physics_stepped':False,'robot_transform_changed':False,'robot_commands_sent':0,
            'camera_motion_only':True,'path_display':'saved actual inference; reveal animation is presentation only',
            'inference_source':str((inference/'result.json').resolve()),'source_sha256':sources,
            'sources_unchanged':True,'full_navigation_success':False,'audit':audit}
    (args.out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('VIDEO_SAVED',str(output_path),flush=True)
except BaseException:
    error=traceback.format_exc(); print(error,flush=True)
    (args.out/'failure.json').write_text(json.dumps({'error':error},indent=2),encoding='utf-8')
finally:
    if writer is not None: writer.close()
    app.close()
