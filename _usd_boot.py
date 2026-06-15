#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_usd_boot.py — make `pxr` importable under C:\\isaacsim\\python.bat WITHOUT
booting Kit/SimulationApp (fast: ~1s vs ~40s). Pure USD layer surgery only.
Usage:  import _usd_boot  # noqa  (must precede `from pxr import ...`)
"""
import sys, os, glob

def _bootstrap():
    if "pxr" in sys.modules:
        return
    cands = glob.glob(r"C:/isaacsim/extscache/omni.usd.libs-*/")
    if not cands:
        return
    p = os.path.abspath(cands[0])
    if p not in sys.path:
        sys.path.insert(0, p)
    for sub in ("", "bin"):
        d = os.path.abspath(os.path.join(p, sub))
        if os.path.isdir(d):
            try:
                os.add_dll_directory(d)
            except Exception:
                pass

_bootstrap()

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass
