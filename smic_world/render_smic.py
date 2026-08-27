#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
render_smic.py — SMIC World 오프스크린 렌더(개념도 대조용).

  C:\\isaacsim\\python.bat render_smic.py                 # 전 카메라
  set SMIC_CAMS=ConceptCam & C:\\isaacsim\\python.bat render_smic.py

노출 레시피는 JM 검증본을 그대로 쓴다: RTX 히스토그램 오토노출 OFF
(안 끄면 흰 설비가 블로우아웃된다). 출력 -> render_smic/<Cam>/rgb_0000.png
"""
import os, sys, json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "render_smic")
ROOT = os.path.join(HERE, "smic_world.usd")
CAMS = [c for c in (os.environ.get("SMIC_CAMS") or
                    "ConceptCam,TopCam,R1Cam,R2Cam").split(",") if c.strip()]
W, H = 1920, 1080

from isaacsim import SimulationApp                                    # noqa: E402
sim = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                     "width": W, "height": H})
import carb.settings                                                  # noqa: E402
import omni.usd                                                       # noqa: E402
from isaacsim.core.utils.extensions import enable_extension           # noqa: E402

enable_extension("omni.replicator.core")
carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)  # 오토노출 OFF
for _ in range(5):
    sim.update()

ctx = omni.usd.get_context()
ctx.open_stage(ROOT)
for _ in range(30):
    sim.update()
stage = ctx.get_stage()

import omni.replicator.core as rep                                    # noqa: E402
rep.orchestrator.set_capture_on_play(False)

done = []
for cam in CAMS:
    path = "/World/" + cam.strip()
    if not stage.GetPrimAtPath(path):
        print("[render] missing camera:", path)
        continue
    d = os.path.join(OUT, cam.strip())
    os.makedirs(d, exist_ok=True)
    rp = rep.create.render_product(path, (W, H))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=d, rgb=True)
    writer.attach([rp])
    for _ in range(60):
        sim.update()
    rep.orchestrator.step(rt_subframes=64)
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()
    for _ in range(5):
        sim.update()
    done.append({"camera": path, "dir": d})
    print("[render] %s -> %s" % (path, d))

with open(os.path.join(HERE, "render_smic_report.json"), "w", encoding="utf-8") as f:
    json.dump({"root": ROOT, "renders": done}, f, ensure_ascii=False, indent=2)
sim.close()
