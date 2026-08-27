#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
world_layers.py — SMIC World 레이어드 합성용 I/O 헬퍼 (JM/서한과 동일 방법론).

월드는 thin 루트 + 6개 서브레이어로 합성된다. 작성 스크립트는 **반드시**
편집 대상 서브레이어를 지정하고 그 레이어만 저장해야 한다.
`stage.Export(ROOT)` 는 전체를 단일 파일로 다시 뭉개므로 절대 금지.

    import _usd_boot                       # 순수 pxr (Kit 없이)
    import world_layers as wl
    stage = Usd.Stage.Open(wl.ROOT)        # Kit 아래에서는 ctx.open_stage(wl.ROOT)
    wl.set_edit_layer(stage, "equipment")  # 이후 authoring -> world_equipment.usd
    ...
    wl.save(stage)                         # dirty 서브레이어만 저장, 루트는 thin 유지

레이어 키: base | equipment | products | labels | robots | staging
WORLD_ROOT_DIR 로 스크래치 사본 경로 지정 가능.
"""
import os

WORLD_DIR = os.environ.get("SMIC_WORLD_DIR", os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.join(WORLD_DIR, "smic_world.usd")

_LAYERS = {
    "base":      "world_base.usd",
    "equipment": "world_equipment.usd",
    "products":  "world_products.usda",
    "labels":    "world_labels.usda",
    "robots":    "world_robots.usda",
    "staging":   "world_staging.usda",
}

# subLayer 순서, 강 -> 약 (ROOT 디렉터리 기준 상대경로)
SUBLAYERS = ["./world_staging.usda", "./world_labels.usda", "./world_products.usda",
             "./world_equipment.usd", "./world_robots.usda", "./world_base.usd"]

LAYER_KEYS = ("base", "equipment", "products", "labels", "robots", "staging")


def layer_path(key):
    return os.path.join(WORLD_DIR, _LAYERS[key])


def find_layer(stage, key):
    """열려 있는 스테이지의 레이어 스택에서 해당 서브레이어를 찾는다."""
    from pxr import Sdf
    fname = _LAYERS[key].lower()
    for lyr in stage.GetLayerStack(False):   # 로컬 레이어 스택(세션 레이어 제외)
        rp = lyr.realPath or lyr.identifier or ""
        if os.path.basename(rp).lower() == fname:
            return lyr
    return Sdf.Layer.FindOrOpen(layer_path(key))


def set_edit_layer(stage, key):
    """스테이지의 edit target 을 지정 서브레이어로 돌린다."""
    from pxr import Usd
    lyr = find_layer(stage, key)
    if lyr is None:
        raise RuntimeError("sublayer '%s' not found in stage layer stack" % key)
    stage.SetEditTarget(Usd.EditTarget(lyr))
    return lyr


def save(stage):
    """dirty 서브레이어만 제자리 저장(flatten 없음).
    루트는 일부러 저장하지 않는다 — Kit 이 stage-open 때 /Render, /World over 를
    루트 edit 레이어에 자동 기록하는데 그게 thin 루트에 구워지면 안 되기 때문."""
    root = stage.GetRootLayer()
    saved = []
    for lyr in stage.GetLayerStack(False):
        if lyr == root:
            continue
        if lyr.dirty:
            lyr.Save()
            saved.append(lyr.identifier)
    return saved


def stamp_root_axis(stage):
    """upAxis/metersPerUnit 은 루트에 있어야 GUI 가 Z-up/미터로 읽는다.
    (JM 함정: 서브레이어만 저장하면 Isaac 기본 Y-up/cm 가 새어 들어와 바닥이 선다.)"""
    from pxr import UsdGeom, Usd
    prev = stage.GetEditTarget()
    stage.SetEditTarget(Usd.EditTarget(stage.GetRootLayer()))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.GetRootLayer().Save()
    stage.SetEditTarget(prev)
