#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
delete_columns.py — 다동 공장 내부의 '중간 철심'(자립형 기둥)을 삭제.
외주(벽선) 기둥은 보존, 바닥 한가운데 서 있는 내부 기둥만 제거.
현재 월드(jm_factory_world_atlas_h1.usd) 직접 편집 + 덮어쓰기. 검증 오버헤드 렌더.

  C:\\isaacsim\\python.bat delete_columns.py

내부 기둥 판정: 14<x<69 그리고 1<y<29 (그리드 외주 14/69/1/29 제외).
대상 경로: /World/Dadong/F1/col_*, /World/Dadong/F2/col_*
"""
import os, sys, json, math

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PROJ = r"C:\Users\USER\ISSAC_SIM_test"
USD  = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
TOP_DIR = os.path.join(PROJ, "render_nocol_top")
REPORT  = os.path.join(PROJ, "delete_columns_report.json")

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                     "width": 1920, "height": 1080})
import carb, carb.settings
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, Sdf

enable_extension("omni.replicator.core")
carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)
for _ in range(5):
    sim.update()

ctx = omni.usd.get_context()
ctx.open_stage(USD)
for _ in range(10):
    sim.update()
stage = ctx.get_stage()

sys.path.insert(0, PROJ); import world_layers as wl
wl.set_edit_layer(stage, "base")  # column deletes target the structural world_base.usd

xc = UsdGeom.XformCache()
report = {"deleted": [], "kept_perimeter": [], "by_floor": {}}

def is_interior(x, y):
    return (14.5 < x < 68.5) and (1.5 < y < 28.5)

for tag in ("F1", "F2"):
    parent = stage.GetPrimAtPath("/World/Dadong/%s" % tag)
    if not parent or not parent.IsValid():
        continue
    dele, kept = 0, 0
    del_paths = []
    for child in parent.GetChildren():
        if not child.GetName().startswith("col_"):
            continue
        t = xc.GetLocalToWorldTransform(child).ExtractTranslation()
        x, y = float(t[0]), float(t[1])
        if is_interior(x, y):
            del_paths.append((str(child.GetPath()), round(x, 2), round(y, 2)))
            dele += 1
        else:
            report["kept_perimeter"].append(str(child.GetPath()))
            kept += 1
    # 실제 삭제(자식 순회 후)
    for p, x, y in del_paths:
        stage.RemovePrim(Sdf.Path(p))
        report["deleted"].append({"path": p, "x": x, "y": y})
    report["by_floor"][tag] = {"deleted": dele, "kept_perimeter": kept}
    carb.log_warn("[delcol] %s deleted=%d kept_perimeter=%d" % (tag, dele, kept))

for _ in range(10):
    sim.update()

wl.save(stage)  # save world_base.usd in place; thin root preserved
report["out_usd"] = USD; report["out_bytes"] = os.path.getsize(wl.layer_path("base"))
report["total_deleted"] = len(report["deleted"])
if os.environ.get("WORLD_NORENDER"):
    with open(REPORT, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    carb.log_warn("[delcol] WORLD_NORENDER -> skip render"); sim.close(); raise SystemExit(0)

# 검증 오버헤드 렌더(기존 FloorCam 사용; 없으면 스킵)
if stage.GetPrimAtPath("/World/FloorCam"):
    import omni.replicator.core as rep
    rep.orchestrator.set_capture_on_play(False)
    os.makedirs(TOP_DIR, exist_ok=True)
    rp = rep.create.render_product("/World/FloorCam", (1920, 1080))
    w = rep.WriterRegistry.get("BasicWriter"); w.initialize(output_dir=TOP_DIR, rgb=True); w.attach([rp])
    for _ in range(110):
        sim.update()
    rep.orchestrator.step(rt_subframes=96)
    rep.orchestrator.wait_until_complete()
    for _ in range(10):
        sim.update()
    report["rendered"] = TOP_DIR

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
carb.log_warn("[delcol] total deleted=%d  report=%s" % (report["total_deleted"], REPORT))
sim.close()
