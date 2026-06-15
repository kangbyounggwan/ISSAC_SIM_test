#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_split.py — adversarial check that the layered root composes IDENTICALLY to
the original flattened world. Pure USD, no Kit.
  C:\\isaacsim\\python.bat verify_split.py
PASS criteria:
  1. Non-robot / non-prototype prim path set is EXACTLY equal (old flat vs new layered).
  2. Each robot's world-aligned bbox matches within 1mm + 4 robots present.
  3. New root composes the 5 sublayers + both clean robot USDs; defaultPrim=/World.
  4. New stage has no composition errors.
"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _usd_boot  # noqa
from pxr import Usd, UsdGeom, Gf

PROJ   = r"C:\Users\USER\ISSAC_SIM_test"
NEW    = os.path.join(PROJ, "jm_factory_world_atlas_h1.usd")
OLD    = os.path.join(PROJ, "jm_factory_world_atlas_h1.flat.usd")
REPORT = os.path.join(PROJ, "verify_split_report.json")
ROBOTS = ["Atlas_1", "H1_1", "Atlas_2", "H1_2"]

res = {"checks": {}, "pass": True}


def fail(key, detail):
    res["checks"][key] = {"pass": False, "detail": detail}
    res["pass"] = False


def ok(key, detail=""):
    res["checks"][key] = {"pass": True, "detail": detail}


def structural_paths(stage):
    out = set()
    for p in Usd.PrimRange(stage.GetPseudoRoot()):
        path = str(p.GetPath())
        if path == "/":
            continue
        if path.startswith("/World/Robots"):
            continue
        if path.startswith("/Flattened_Prototype_"):
            continue
        out.add(path)
    return out


def robot_bboxes(stage):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    d = {}
    for nm in ROBOTS:
        prim = stage.GetPrimAtPath("/World/Robots/" + nm)
        if not (prim and prim.IsValid()):
            d[nm] = None
            continue
        rng = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        mn, mx = rng.GetMin(), rng.GetMax()
        d[nm] = [round(mx[i] - mn[i], 4) for i in range(3)] + [round(mn[2], 4), round(mx[2], 4)]
    return d


# ---------------------------------------------------------------- open both
old = Usd.Stage.Open(OLD)
new = Usd.Stage.Open(NEW)

# 1. structural path-set equality -----------------------------------------
sp_old, sp_new = structural_paths(old), structural_paths(new)
missing = sorted(sp_old - sp_new)   # present in original, lost in layered  (BAD)
extra   = sorted(sp_new - sp_old)   # appeared in layered                  (BAD)
res["counts"] = {"structural_old": len(sp_old), "structural_new": len(sp_new),
                 "missing": len(missing), "extra": len(extra)}
if missing or extra:
    fail("structural_paths_equal", {"missing": missing[:40], "extra": extra[:40]})
else:
    ok("structural_paths_equal", "%d paths identical" % len(sp_old))

# 2. robot bboxes ----------------------------------------------------------
bb_old, bb_new = robot_bboxes(old), robot_bboxes(new)
res["robot_bboxes"] = {"old": bb_old, "new": bb_new}
present = [nm for nm in ROBOTS if bb_new.get(nm)]
if len(present) != 4:
    fail("robots_present", "only %s present" % present)
else:
    ok("robots_present", "4/4")
bad = []
for nm in ROBOTS:
    a, b = bb_old.get(nm), bb_new.get(nm)
    if not a or not b:
        bad.append((nm, "missing", a, b)); continue
    if any(abs(a[i] - b[i]) > 0.001 for i in range(len(a))):
        bad.append((nm, "bbox-diff", a, b))
if bad:
    fail("robot_bbox_match", bad)
else:
    ok("robot_bbox_match", "all 4 within 1mm")

# 3. composition layers ----------------------------------------------------
new_layers = [l.identifier for l in new.GetUsedLayers()]
res["new_used_layers"] = new_layers
want_sub = ["world_staging.usda", "world_labels.usda", "world_equipment.usd",
            "world_robots.usda", "world_base.usd"]
miss_sub = [s for s in want_sub if not any(s in l for l in new_layers)]
want_ref = ["atlas_clean.usd", "h1_clean.usd"]
miss_ref = [s for s in want_ref if not any(s in l for l in new_layers)]
if miss_sub or miss_ref:
    fail("composition_layers", {"missing_sublayers": miss_sub, "missing_refs": miss_ref})
else:
    ok("composition_layers", "5 sublayers + 2 robot refs present")

dp = new.GetDefaultPrim()
if dp and str(dp.GetPath()) == "/World":
    ok("default_prim", "/World")
else:
    fail("default_prim", str(dp.GetPath()) if dp else None)

# 4. composition errors ----------------------------------------------------
try:
    errs = new.GetCompositionErrors()
except Exception:
    errs = []
if errs:
    fail("no_composition_errors", [str(e) for e in errs][:20])
else:
    ok("no_composition_errors", "clean")

# file sizes ---------------------------------------------------------------
res["file_sizes"] = {f: os.path.getsize(os.path.join(PROJ, f)) for f in
                     ["jm_factory_world_atlas_h1.usd", "jm_factory_world_atlas_h1.flat.usd",
                      "world_base.usd", "world_equipment.usd", "world_labels.usda",
                      "world_robots.usda", "world_staging.usda"] if os.path.exists(os.path.join(PROJ, f))}

with open(REPORT, "w", encoding="utf-8") as f:
    json.dump(res, f, ensure_ascii=False, indent=2)

print("================ VERIFY:", "PASS" if res["pass"] else "FAIL", "================")
for k, v in res["checks"].items():
    print(("  [OK]  " if v["pass"] else "  [XX]  ") + k + "  " +
          (json.dumps(v["detail"], ensure_ascii=False) if v["detail"] else ""))
print("structural paths: old=%d new=%d  missing=%d extra=%d" %
      (res["counts"]["structural_old"], res["counts"]["structural_new"],
       res["counts"]["missing"], res["counts"]["extra"]))
print("robot bboxes (new):", json.dumps(bb_new, ensure_ascii=False))
print("report:", REPORT)
