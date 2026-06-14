#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""render_dadong_3f.py — 다동 3층 장비 배치 디지털화 검증용 평면 렌더(PIL).
원 도면과 나란히 비교해 위치/치수를 확인·보정하기 위한 top-down PNG.
출력: dadong_3f_layout.png"""
from PIL import Image, ImageDraw, ImageFont
from dadong_3f_layout import EQUIPMENT, GRID, ZONE_COLOR, FLOW

PX = 14                       # px per grid cell
COLS, ROWS = GRID["cols"], GRID["rows"]
MX, MY = 44, 30               # margin (axis labels)
W = MX + COLS * PX + 14
H = MY + ROWS * PX + 14


def font(sz, bold=False):
    for p in (r"C:\Windows\Fonts\malgunbd.ttf" if bold else r"C:\Windows\Fonts\malgun.ttf",
              r"C:\Windows\Fonts\malgun.ttf"):
        try:
            return ImageFont.truetype(p, sz)
        except Exception:
            pass
    return ImageFont.load_default()


F_LBL, F_DIM, F_AX, F_TTL = font(11, True), font(9), font(9), font(16, True)


def gx(c):
    return MX + (c - 1) * PX


def gy(r):
    return MY + (r - 1) * PX


im = Image.new("RGB", (W, H), (255, 255, 255))
d = ImageDraw.Draw(im)

# 존 배경 틴트(참고): 남동 룸 / 복도
d.rectangle([gx(67), gy(24), gx(101), gy(55)], fill=(228, 240, 228))   # 남동 공정룸(녹색)
d.rectangle([gx(7), gy(25), gx(67), gy(55)], fill=(255, 247, 170))     # 복도(노랑)
# 코어(좌/우)
for cc in ([1, 4], [104, 110]):
    d.rectangle([gx(cc[0]), gy(24), gx(cc[1]), gy(28)], fill=(20, 20, 20))   # EV
    d.rectangle([gx(cc[0]), gy(29), gx(cc[1]), gy(43)], fill=(225, 225, 225), outline=(120, 120, 120))  # 계단
    d.rectangle([gx(cc[0]), gy(45), gx(cc[1]), gy(55)], fill=(214, 196, 184), outline=(120, 120, 120))  # 화장실

# 그리드
for c in range(0, COLS + 1):
    col = (170, 170, 170) if c % 10 == 0 else (232, 232, 232)
    d.line([gx(c + 1), MY, gx(c + 1), MY + ROWS * PX], fill=col)
    if c % 5 == 0 and c <= COLS:
        d.text((gx(c + 1) - 5, MY + ROWS * PX + 2), str(c if c else 1), font=F_AX, fill=(90, 90, 90))
for r in range(0, ROWS + 1):
    col = (170, 170, 170) if r % 10 == 0 else (232, 232, 232)
    d.line([MX, gy(r + 1), MX + COLS * PX, gy(r + 1)], fill=col)
    if r % 5 == 0 and r <= ROWS:
        d.text((MX + COLS * PX + 3, gy(r + 1) - 6), str(r if r else 1), font=F_AX, fill=(90, 90, 90))

# 건물 외곽
d.rectangle([gx(1), gy(1), gx(COLS + 1), gy(ROWS + 1)], outline=(40, 120, 60), width=3)

cen = {}   # id -> (px,py) for flow arrows


def box(eid, kr, w, d_, cc, cr, zone):
    cw, ch = w / GRID["cell"], d_ / GRID["cell"]
    l, t = gx(cc - cw / 2 + 1), gy(cr - ch / 2 + 1)
    r, b = l + cw * PX, t + ch * PX
    col = tuple(int(v * 255) for v in ZONE_COLOR[zone])
    outline = (60, 60, 60) if zone != "reagent" else (200, 40, 40)
    d.rectangle([l, t, r, b], fill=col, outline=outline, width=1)
    cen[eid] = ((l + r) / 2, (t + b) / 2)
    # 라벨
    long = max(w, d_)
    lbl = kr
    bb = d.textbbox((0, 0), lbl, font=F_LBL)
    if bb[2] - bb[0] <= (r - l) - 2:
        d.text(((l + r) / 2 - (bb[2] - bb[0]) / 2, (t + b) / 2 - 11), lbl, font=F_LBL, fill=(20, 20, 20))
        if long >= 2.0:
            dim = f"{w:g}×{d_:g}"
            db = d.textbbox((0, 0), dim, font=F_DIM)
            d.text(((l + r) / 2 - (db[2] - db[0]) / 2, (t + b) / 2 + 1), dim, font=F_DIM, fill=(150, 30, 30))


for (eid, kr, en, w, d_, cc, cr, zone) in EQUIPMENT:
    box(eid, kr, w, d_, cc, cr, zone)

# 공정 흐름 화살표
for a, b in FLOW:
    if a in cen and b in cen:
        (x0, y0), (x1, y1) = cen[a], cen[b]
        d.line([x0, y0, x1, y1], fill=(120, 120, 120), width=2)

d.text((MX, 6), "다동 3층 장비 배치 — 디지털화 검증 (grid 0.5m/cell, 110×56 = 55×28m)",
       font=F_TTL, fill=(20, 20, 20))
im.save("dadong_3f_layout.png")
print("wrote dadong_3f_layout.png", im.size, "| equipment:", len(EQUIPMENT))
