#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""preview.py — pxr 없이 jm_factory_world 스펙을 PIL 로 평면/단면 렌더(좌표·치수 검증용)."""
from PIL import Image, ImageDraw
import world_geometry as W

SITE, DA, NA, GA, C = W.SITE, W.DADONG, W.NADONG, W.GADONG, W.C
RW, RE = W.ROAD_W_WEST, W.ROAD_W_EAST


def col(c):
    return tuple(int(round(v * 255)) for v in c)


# ---------------- 평면도 (top-down, X=동 / Y=북) ----------------
def plan(path):
    margin = 60
    minx, maxx = -RW - 3, SITE["w"] + RE + 3
    miny, maxy = -3, SITE["d"] + 3
    sc = 9.0  # px per meter
    Wpx = int((maxx - minx) * sc) + 2 * margin
    Hpx = int((maxy - miny) * sc) + 2 * margin
    img = Image.new("RGB", (Wpx, Hpx), (250, 250, 248))
    d = ImageDraw.Draw(img)

    def X(x): return margin + (x - minx) * sc
    def Y(y): return Hpx - margin - (y - miny) * sc   # 북쪽이 위로

    def rect(x0, y0, x1, y1, fill=None, outline=(40, 40, 40), w=1):
        d.rectangle([X(x0), Y(y1), X(x1), Y(y0)], fill=fill, outline=outline, width=w)

    # 도로 / 대지 / 야드
    rect(-RW, 0, 0, SITE["d"], fill=col(C["road"]), outline=None)
    rect(SITE["w"], 0, SITE["w"] + RE, SITE["d"], fill=col(C["road"]), outline=None)
    rect(0, 0, SITE["w"], SITE["d"], fill=(228, 228, 222), outline=(90, 90, 90), w=2)

    # 다동
    rect(DA["x0"], DA["y0"], DA["x1"], DA["y1"], fill=col(C["da_wall"]), outline=(30, 30, 30), w=2)
    for gx in DA["grid_x"]:
        for gy in DA["grid_y"]:
            d.ellipse([X(gx) - 3, Y(gy) - 3, X(gx) + 3, Y(gy) + 3], fill=(40, 40, 40))
    for cr in DA["cores"]:
        rect(cr["x0"], cr["y0"], cr["x1"], cr["y1"], fill=col(C["core"]), outline=(20, 20, 20), w=2)
    # 나동 / 가동
    rect(NA["x0"], NA["y0"], NA["x1"], NA["y1"], fill=col(C["na_wall"]), outline=(30, 30, 30), w=2)
    rect(GA["x0"], GA["y0"], GA["x1"], GA["y1"], fill=col(C["ga_wall"]), outline=(30, 30, 30), w=2)

    # 라벨
    d.text((X(38), Y(15)), "DADONG (main factory) 55x28, 2F", fill=(0, 0, 0))
    d.text((X(40), Y(53)), "NADONG (factory) 42x17, 1F", fill=(0, 0, 0))
    d.text((X(4.5), Y(53)), "GADONG\n10x17\n3F", fill=(0, 0, 0))
    d.text((X(-RW + 2), Y(33)), "20m ROAD", fill=(230, 230, 230))
    d.text((X(SITE["w"] + 1), Y(33)), "10m\nROAD", fill=(230, 230, 230))
    # 방위 + 치수
    d.text((margin, 12), "PLAN (top=North)  site 75 x 66 m   [px/m=%.0f]" % sc, fill=(0, 0, 0))
    d.line([X(2), Y(63), X(2), Y(63) - 30], fill=(200, 0, 0), width=2)
    d.text((X(2) - 4, Y(63) - 44), "N", fill=(200, 0, 0))
    img.save(path)
    print("saved", path, img.size)


# ---------------- 단면(N-S, 가동/나동/다동 높이 비교) ----------------
def section(path):
    margin = 60
    miny, maxy = -3, SITE["d"] + 3
    maxz = 14.0
    sc = 11.0
    Wpx = int((maxy - miny) * sc) + 2 * margin
    Hpx = int((maxz + 2) * sc) + 2 * margin
    img = Image.new("RGB", (Wpx, Hpx), (250, 250, 248))
    d = ImageDraw.Draw(img)

    def Yp(y): return margin + (y - miny) * sc
    def Z(z): return Hpx - margin - z * sc

    def poly(pts, fill, outline=(30, 30, 30)):
        d.polygon([(Yp(y), Z(z)) for (y, z) in pts], fill=fill, outline=outline)

    # 지면
    d.line([Yp(miny), Z(0), Yp(maxy), Z(0)], fill=(60, 60, 60), width=2)

    # 가동 (y45-62, 3F 평지붕)
    poly([(GA["y0"], 0), (GA["y0"], GA["roof"]), (GA["y1"], GA["roof"]), (GA["y1"], 0)], col(C["ga_wall"]))
    for fz in GA["floors"][1:] + [GA["roof"]]:
        d.line([Yp(GA["y0"]), Z(fz), Yp(GA["y1"]), Z(fz)], fill=(80, 80, 80), width=1)
    d.text((Yp(GA["y0"]), Z(GA["roof"]) - 14), "GADONG 3F (z=9.9)", fill=(0, 0, 0))

    # 나동 (y45-62, 박공 6/7.5) — 동일 y대역이라 살짝 겹침: 점선 표기
    nym = (NA["y0"] + NA["y1"]) / 2
    poly([(NA["y0"], 0), (NA["y0"], NA["eave"]), (nym, NA["ridge"]),
          (NA["y1"], NA["eave"]), (NA["y1"], 0)], None, outline=(150, 60, 40))
    d.text((Yp(NA["y0"]) + 30, Z(NA["ridge"]) - 12), "NADONG ridge 7.5 (dashed)", fill=(150, 60, 40))

    # 다동 (y1-29, 박공 처마10/용마루12.4, 2F z=6)
    dym = (DA["y0"] + DA["y1"]) / 2
    poly([(DA["y0"], 0), (DA["y0"], DA["eave"]), (dym, DA["ridge"]),
          (DA["y1"], DA["eave"]), (DA["y1"], 0)], col(C["da_wall"]))
    d.line([Yp(DA["y0"]), Z(DA["f2"]), Yp(DA["y1"]), Z(DA["f2"])], fill=(80, 80, 80), width=2)
    d.line([Yp(DA["y0"]), Z(DA["eave"]), Yp(DA["y1"]), Z(DA["eave"])], fill=(120, 120, 120), width=1)
    d.text((Yp(DA["y0"]) + 4, Z(DA["f2"]) - 14), "2F z=6", fill=(0, 0, 0))
    d.text((Yp(dym) - 30, Z(DA["ridge"]) - 14), "DADONG ridge 12.4 / eave 10", fill=(0, 0, 0))

    # z 눈금
    for zz in range(0, 14, 2):
        d.line([Yp(miny) - 5, Z(zz), Yp(miny), Z(zz)], fill=(0, 0, 0))
        d.text((Yp(miny) - 28, Z(zz) - 6), "%dm" % zz, fill=(0, 0, 0))
    d.text((margin, 12), "SECTION  (left=South y=0  ->  right=North y=66)", fill=(0, 0, 0))
    img.save(path)
    print("saved", path, img.size)


if __name__ == "__main__":
    plan(r"C:\issac sim test\preview_plan.png")
    section(r"C:\issac sim test\preview_section.png")
