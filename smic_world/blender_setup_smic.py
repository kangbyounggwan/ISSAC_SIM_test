#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
blender_setup_smic.py — SMIC World 를 **블렌더 안에서** 실행하는 셋업 스크립트.

Blender GUI:  Scripting 탭 > Open > 이 파일 > Run Script
CLI       :  "C:\\Program Files\\Blender Foundation\\Blender 4.5\\blender.exe" ^
                 --python blender_setup_smic.py -- --usd

왜 필요한가 (Blender 4.5.1 에서 실측)
  USD 를 그냥 임포트하면 카메라 4대와 조명은 잘 들어오는데, 메시 556개 중
  **529개가 머티리얼 없이 회색**으로 들어온다. 이 월드의 색은 UsdShade 머티리얼이
  아니라 `displayColor` 프리미티브 속성이라 블렌더가 컬러 **어트리뷰트**로만 읽기
  때문이다. 이 스크립트가 Attribute(displayColor) -> Base Color 머티리얼을 만들어
  머티리얼 없는 메시 전부에 물린다.

옵션 (`--` 뒤에)
  --usd            USD 임포트 (기본). 카메라/조명/계층까지 들어온다
  --obj            OBJ 임포트. MTL 색이 확실하지만 카메라/조명은 없다
  --both           USD(카메라·조명) + OBJ(형상·색). 좌표계가 같아 정확히 겹친다
  --no-robots      OBJ 경량본 사용
  --cam <이름>     활성 카메라 (기본 ConceptCam)
"""
import os, sys
import bpy

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else os.getcwd()
USD = os.path.join(HERE, "smic_world.usd")
OBJ_FULL = os.path.join(HERE, "export", "smic_world.obj")
OBJ_LIGHT = os.path.join(HERE, "export", "smic_world_norobots.obj")

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
MODE = "both" if "--both" in argv else ("obj" if "--obj" in argv else "usd")
NO_ROBOTS = "--no-robots" in argv
CAM = argv[argv.index("--cam") + 1] if "--cam" in argv else "ConceptCam"


def displaycolor_material():
    """displayColor 컬러 어트리뷰트를 Base Color 로 읽는 머티리얼."""
    name = "SMIC_displayColor"
    m = bpy.data.materials.get(name)
    if m:
        return m
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    bsdf = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
    attr = nt.nodes.new("ShaderNodeAttribute")
    attr.attribute_type = "GEOMETRY"
    attr.attribute_name = "displayColor"
    attr.location = (-320, 200)
    if bsdf:
        nt.links.new(attr.outputs["Color"], bsdf.inputs["Base Color"])
        if "Roughness" in bsdf.inputs:
            bsdf.inputs["Roughness"].default_value = 0.55
    return m


def apply_displaycolor():
    m = displaycolor_material()
    n = 0
    for o in bpy.data.objects:
        if o.type != "MESH" or o.data.materials:
            continue
        if "displayColor" not in [a.name for a in o.data.attributes]:
            continue
        o.data.materials.append(m)
        n += 1
    return n


def normalize_lights():
    """USD 조명은 Isaac RTX 강도(Sun 1600 / RectLight 2600) 그대로 들어와서
    Cycles·EEVEE 에서 **완전 화이트아웃**된다(실측 확인). 블렌더 물리 단위로 다시 잡는다.
      Sun  = W/m^2 (irradiance),  Area = W (총 복사속)
    DomeLight(Sky)는 라이트 오브젝트로 안 들어오므로 월드 배경으로 대신한다."""
    n = 0
    for o in bpy.data.objects:
        if o.type != "LIGHT":
            continue
        d = o.data
        if d.type == "SUN":
            d.energy = 3.0
            d.angle = 0.0093            # 약 0.53도
        elif d.type == "AREA":
            d.energy = 300.0
        elif d.type == "POINT":
            d.energy = 100.0
        elif d.type == "SPOT":
            d.energy = 200.0
        n += 1

    # 월드 배경 = 하늘광 대용 (Isaac DomeLight 300 에 대응)
    w = bpy.data.worlds.get("SMIC_Sky") or bpy.data.worlds.new("SMIC_Sky")
    w.use_nodes = True
    bg = next((x for x in w.node_tree.nodes if x.type == "BACKGROUND"), None)
    if bg:
        bg.inputs["Color"].default_value = (0.58, 0.63, 0.70, 1.0)
        bg.inputs["Strength"].default_value = 0.6
    bpy.context.scene.world = w
    return n


def normalize_cameras():
    """블렌더 USD 임포터는 카메라 focalLength/aperture 를 **100배**로 들여온다
    (실측: 우리 16 mm / 20.955 mm -> 1600 / 2095.5). FOV 비율은 보존되지만
    초점거리가 1600 mm 가 되어 피사계심도가 사실상 0 → 화면 전체가 뭉개진다.
    실제 mm 로 되돌리고 DOF 를 끈다. 위치·방향은 정확히 들어오므로 건드리지 않는다."""
    out = []
    for o in bpy.data.objects:
        if o.type != "CAMERA":
            continue
        d = o.data
        if d.sensor_width > 100.0:                 # 100배 스케일 흔적
            s = 100.0
            d.lens /= s
            d.sensor_width /= s
            if hasattr(d, "sensor_height"):
                d.sensor_height /= s
        d.sensor_fit = "HORIZONTAL"                # aperture 가 수평 기준
        if hasattr(d, "dof"):
            d.dof.use_dof = False
        out.append((o.name, round(d.lens, 2), round(d.sensor_width, 3)))
    return out


def drop_factory_startup():
    """기본 시작 씬(Cube/Camera/Light)만 있으면 지운다. 그대로 두면 기본 Point 라이트가
    렌더를 오염시킨다. 사용자가 만든 오브젝트가 하나라도 있으면 **아무것도 안 지운다**."""
    objs = list(bpy.data.objects)
    if not objs or len(objs) > 3 or any(o.name not in ("Cube", "Camera", "Light") for o in objs):
        return 0
    for o in objs:
        bpy.data.objects.remove(o, do_unlink=True)
    return len(objs)


def main():
    print("=== SMIC World -> Blender (mode=%s) ===" % MODE)
    n_drop = drop_factory_startup()
    if n_drop:
        print("  removed %d factory-startup objects (Cube/Camera/Light)" % n_drop)
    if MODE in ("usd", "both"):
        if not os.path.exists(USD):
            sys.exit("[blender] missing %s" % USD)
        bpy.ops.wm.usd_import(filepath=USD, import_cameras=True, import_lights=True,
                              import_materials=True, import_visible_only=True, scale=1.0)
        print("  usd imported:", len(bpy.data.objects), "objects")

    if MODE in ("obj", "both"):
        obj = OBJ_LIGHT if NO_ROBOTS else OBJ_FULL
        if not os.path.exists(obj):
            sys.exit("[blender] missing %s — run export_smic_obj.py first" % obj)
        # 내보내기가 `o` 를 쓰므로 기본값(use_split_objects=True)으로 프림별 분리된다.
        bpy.ops.wm.obj_import(filepath=obj, forward_axis="NEGATIVE_Z", up_axis="Y",
                              use_split_objects=True, global_scale=1.0)
        print("  obj imported:", os.path.basename(obj))

    n = apply_displaycolor()
    print("  displayColor material applied to %d meshes" % n)
    if "--keep-lights" not in argv:
        print("  lights normalized to Blender units:", normalize_lights())
    cams = normalize_cameras()
    if cams:
        print("  cameras normalized (name, lens_mm, sensor_mm):", cams)

    # 렌더 설정: 카메라 aperture 가 1920x1080 기준이라 해상도를 맞춰야 화각이 맞는다
    sc = bpy.context.scene
    sc.render.resolution_x, sc.render.resolution_y = 1920, 1080
    sc.render.resolution_percentage = 100
    cam = bpy.data.objects.get(CAM)
    if cam and cam.type == "CAMERA":
        sc.camera = cam
        print("  active camera:", CAM)
    else:
        print("  camera %r not found (OBJ 모드에는 카메라가 없다)" % CAM)

    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    nomat = [o for o in meshes if not o.data.materials]
    print("  meshes=%d  without material=%d" % (len(meshes), len(nomat)))
    print("  cameras:", [o.name for o in bpy.data.objects if o.type == "CAMERA"])
    print("  lights :", [o.name for o in bpy.data.objects if o.type == "LIGHT"])
    print("=== done ===")


main()
