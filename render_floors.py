#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""render_floors.py — 층별 구조를 패널 몽타주로 렌더(PIL). 각 층을 따로 3D로 보여줌."""
from PIL import Image, ImageDraw
from render_iso import D, U, V, LIGHT, face_normal, dot, BOX_FACES
from world_geometry import build_scene, box_corners

PANELS = [
    ("__all__", "전체 단지"),
    ("DA-1F", "다동 1층 · 공장(개방홀+코어)"),
    ("DA-2F", "다동 2층 · 공장+2F철골거더"),
    ("DA-roof", "다동 지붕 · 박공+철골트러스"),
    ("GA-1F", "가동 1층 · 주방/식당"),
    ("GA-2F", "가동 2층 · 사무실"),
    ("GA-3F", "가동 3층 · 기숙사"),
    ("NA-1F", "나동 1층 · 공장"),
]


def panel(parts, floor, w, h):
    sub = parts if floor == "__all__" else [p for p in parts if p["floor"] == floor]
    faces = []
    for p in sub:
        col = p["color"]
        if p["type"] == "box":
            verts = box_corners(p)
            polys = [[verts[i] for i in f] for f in BOX_FACES]
        else:
            polys = [[p["points"][i] for i in f] for f in p["faces"]]
        for poly in polys:
            nrm = face_normal(poly)
            if dot(nrm, D) > -0.02:
                continue
            sh = 0.34 + 0.66 * max(0.0, dot(nrm, [-l for l in LIGHT]))
            fill = tuple(min(255, int(c * 255 * sh)) for c in col)
            spts = [(dot(v, U), dot(v, V)) for v in poly]
            depth = sum(dot(v, D) for v in poly) / len(poly)
            faces.append((depth, spts, fill))
    img = Image.new("RGB", (w, h), (176, 188, 200))
    if not faces:
        return img
    faces.sort(key=lambda f: -f[0])
    allx = [x for _, sp, _ in faces for x, _ in sp]
    ally = [y for _, sp, _ in faces for _, y in sp]
    minx, maxx, miny, maxy = min(allx), max(allx), min(ally), max(ally)
    pad = 26
    sc = min((w - 2 * pad) / max(maxx - minx, .1), (h - 2 * pad - 16) / max(maxy - miny, .1))
    ox = (w - (maxx - minx) * sc) / 2 - minx * sc
    oy = h - pad - (0 - miny) * sc
    d = ImageDraw.Draw(img)
    for _, sp, fill in faces:
        d.polygon([(ox + x * sc, oy - y * sc) for x, y in sp], fill=fill, outline=(40, 43, 48))
    return img


def render(path):
    parts = build_scene()
    pw, ph = 620, 430
    cols = 4
    rows = (len(PANELS) + cols - 1) // cols
    lab = 20
    sheet = Image.new("RGB", (cols * pw, rows * (ph + lab)), (250, 250, 248))
    dr = ImageDraw.Draw(sheet)
    for i, (fl, title) in enumerate(PANELS):
        im = panel(parts, fl, pw, ph)
        cx, cy = i % cols, i // cols
        x0, y0 = cx * pw, cy * (ph + lab)
        sheet.paste(im, (x0, y0 + lab))
        dr.rectangle([x0, y0, x0 + pw - 1, y0 + ph + lab - 1], outline=(180, 184, 188))
        dr.text((x0 + 8, y0 + 4), title, fill=(15, 25, 40))
    sheet.save(path)
    print("saved", path, sheet.size)


if __name__ == "__main__":
    render(r"C:\issac sim test\render_floors.png")
