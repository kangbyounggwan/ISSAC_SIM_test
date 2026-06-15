#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
world_layers.py — layer-aware I/O helpers for the layered JM factory world.

The world is composed from sublayers (see split_world.py). Authoring scripts must
route their edits into the RIGHT sublayer and Save (NOT stage.Export, which would
re-flatten everything back into one 24MB monolith). Usage in a script:

    import world_layers as wl
    ctx.open_stage(wl.ROOT)
    stage = ctx.get_stage()
    wl.set_edit_layer(stage, "equipment")   # subsequent authoring -> world_equipment.usd
    ...build/RemovePrim...
    wl.set_edit_layer(stage, "staging")      # cameras/lights -> world_staging.usda
    ...make_cam...
    wl.save(stage)                           # save dirty sublayers in place; root stays thin

Layer keys: base | equipment | labels | robots | staging
Set WORLD_ROOT_DIR to point at a scratch copy of the layer set (for safe testing).
"""
import os

WORLD_DIR = os.environ.get("WORLD_ROOT_DIR", r"C:\Users\USER\ISSAC_SIM_test")
ROOT = os.path.join(WORLD_DIR, "jm_factory_world_atlas_h1.usd")

_LAYERS = {
    "base":      "world_base.usd",
    "equipment": "world_equipment.usd",
    "labels":    "world_labels.usda",
    "robots":    "world_robots.usda",
    "staging":   "world_staging.usda",
}


def layer_path(key):
    return os.path.join(WORLD_DIR, _LAYERS[key])


def find_layer(stage, key):
    """Return the Sdf.Layer for a sublayer key from the open stage's layer stack."""
    from pxr import Sdf
    fname = _LAYERS[key].lower()
    for lyr in stage.GetLayerStack(False):  # local layer stack, no session layers
        rp = lyr.realPath or lyr.identifier or ""
        if os.path.basename(rp).lower() == fname:
            return lyr
    return Sdf.Layer.FindOrOpen(layer_path(key))


def set_edit_layer(stage, key):
    """Point the stage's edit target at the given sublayer; returns the layer."""
    from pxr import Usd
    lyr = find_layer(stage, key)
    if lyr is None:
        raise RuntimeError("sublayer '%s' not found in stage layer stack" % key)
    stage.SetEditTarget(Usd.EditTarget(lyr))
    return lyr


def save(stage):
    """Persist by saving the dirty SUBLAYERS in place (no flatten). The root layer
    is deliberately NOT saved: Kit auto-authors /Render + /World overs into the
    root edit layer at stage-open, and we don't want those baked into the thin
    root on disk. All real content was routed to sublayers via set_edit_layer."""
    root = stage.GetRootLayer()
    saved = []
    for lyr in stage.GetLayerStack(False):  # local layer stack, no session layers
        if lyr == root:
            continue
        if lyr.dirty:
            lyr.Save()
            saved.append(lyr.identifier)
    return saved
