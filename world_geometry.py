#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
world_geometry.py — 제이엠일렉트로닉스 시화공장 월드 **단일 지오메트리 소스**(pxr 무관).

build_scene() -> parts 리스트. USD / OBJ / three.js 뷰어 / PIL 렌더가 공유.
단위 m, Z-up, 원점=대지 남서 모서리(+X 동, +Y 북).

각 part 에 floor(층) / cat(arch|struct|site) 태그가 있어 **층 선택·구조/건축 토글**이 가능.

part:
  box : {type,'box', path, center, size, color, floor, cat, collider, rot(quat xyzw|None)}
  mesh: {type,'mesh',path, points, faces, color, floor, cat, collider}

floor 코드:
  site / DA-1F DA-2F DA-roof / NA-1F NA-roof / GA-1F GA-2F GA-3F GA-roof
"""
import math

# ------------------------------------------------------------------ 마스터 스펙(m)
SITE = dict(w=75.0, d=66.0)
ROAD_W_WEST = 20.0
ROAD_W_EAST = 10.0

DADONG = dict(
    x0=14.0, x1=69.0, y0=1.0, y1=29.0,
    f1=0.0, f2=6.0, eave=10.0, ridge=12.4,
    grid_x=[14.0 + 5.0 * i for i in range(12)],
    grid_y=[1.0, 8.0, 15.0, 22.0, 29.0],
    col=0.30, wall=0.20,
    cores=[dict(x0=14.0, x1=19.0, y0=8.0, y1=22.0),
           dict(x0=64.0, x1=69.0, y0=8.0, y1=22.0)],
)
NADONG = dict(x0=22.0, x1=64.0, y0=45.0, y1=62.0, f1=0.0, eave=6.0, ridge=7.5, wall=0.20)
GADONG = dict(x0=4.0, x1=14.0, y0=45.0, y1=62.0,
              floors=[0.0, 3.3, 6.6], fh=3.3, roof=9.9, parapet=0.6, wall=0.18)

C = dict(
    ground=(0.46, 0.47, 0.44), road=(0.27, 0.28, 0.30), yard=(0.60, 0.60, 0.57),
    da_wall=(0.74, 0.76, 0.80), da_roof=(0.27, 0.40, 0.62), core=(0.86, 0.84, 0.79),
    column=(0.50, 0.52, 0.55), slab=(0.66, 0.66, 0.63), beam=(0.40, 0.45, 0.52),
    truss=(0.46, 0.36, 0.30),
    na_wall=(0.80, 0.78, 0.72), na_roof=(0.55, 0.40, 0.34),
    ga_wall=(0.87, 0.85, 0.80), ga_roof=(0.52, 0.52, 0.52),
    part_din=(0.86, 0.74, 0.55), part_off=(0.70, 0.80, 0.74), part_dorm=(0.80, 0.72, 0.78),
    kitchen=(0.78, 0.80, 0.84), glass=(0.50, 0.70, 0.85), fence=(0.42, 0.42, 0.42),
)

# 층 표시 순서/라벨 (뷰어 UI)
FLOORS = [
    ("site", "대지·외부"),
    ("DA-1F", "다동 1층 · 공장"),
    ("DA-2F", "다동 2층 · 공장"),
    ("DA-roof", "다동 지붕·철골"),
    ("NA-1F", "나동 1층 · 공장"),
    ("NA-roof", "나동 지붕"),
    ("GA-1F", "가동 1층 · 주방/식당"),
    ("GA-2F", "가동 2층 · 사무실"),
    ("GA-3F", "가동 3층 · 기숙사"),
    ("GA-roof", "가동 옥상"),
]


# ------------------------------------------------------------------ 쿼터니언 유틸
def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _quat_dir(dx, dy, dz):
    """로컬 +X 축을 (dx,dy,dz) 방향으로 회전시키는 쿼터니언(xyzw). yaw(Z)·pitch(Y)."""
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(dz, math.hypot(dx, dy))
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    qy = (0.0, math.sin(-pitch / 2), 0.0, math.cos(-pitch / 2))
    return _qmul(qz, qy)


def quat_matrix(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


_BOX_V = [(-.5, -.5, -.5), (.5, -.5, -.5), (.5, .5, -.5), (-.5, .5, -.5),
          (-.5, -.5, .5), (.5, -.5, .5), (.5, .5, .5), (-.5, .5, .5)]


def box_corners(part):
    """box part 의 월드 8코너(회전 포함). OBJ/PIL 렌더 공용."""
    cx, cy, cz = part["center"]; sx, sy, sz = part["size"]
    q = part.get("rot")
    if q:
        m = quat_matrix(q)
        out = []
        for vx, vy, vz in _BOX_V:
            lx, ly, lz = vx * sx, vy * sy, vz * sz
            wx = m[0][0] * lx + m[0][1] * ly + m[0][2] * lz
            wy = m[1][0] * lx + m[1][1] * ly + m[1][2] * lz
            wz = m[2][0] * lx + m[2][1] * ly + m[2][2] * lz
            out.append((cx + wx, cy + wy, cz + wz))
        return out
    return [(cx + vx * sx, cy + vy * sy, cz + vz * sz) for (vx, vy, vz) in _BOX_V]


# ================================================================== 씬 빌더
def build_scene():
    parts = []
    cur = {"floor": "site", "cat": "arch"}

    def F(floor, cat):
        cur["floor"] = floor; cur["cat"] = cat

    def box(path, center, size, color, collider=True, rot=None, floor=None, cat=None):
        parts.append(dict(type="box", path=path, center=tuple(center), size=tuple(size),
                          color=tuple(color), floor=floor or cur["floor"], cat=cat or cur["cat"],
                          collider=collider, rot=rot))

    def mesh(path, points, faces, color, collider=True, floor=None, cat=None):
        parts.append(dict(type="mesh", path=path, points=[tuple(p) for p in points],
                          faces=[list(f) for f in faces], color=tuple(color),
                          floor=floor or cur["floor"], cat=cat or cur["cat"], collider=collider))

    def member(path, p0, p1, w, h, color, collider=False):
        """양 끝점 p0,p1(3D) 사이 단면 w×h 부재(보/거더/래프터). 회전 자동."""
        x0, y0, z0 = p0; x1, y1, z1 = p1
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        L = math.sqrt(dx * dx + dy * dy + dz * dz) or 0.01
        rot = _quat_dir(dx, dy, dz)
        box(path, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (L, w, h), color,
            collider=collider, rot=rot)

    def wall(path, p0, p1, height, base_z=0.0, thick=0.2, openings=(), color=C["da_wall"], collider=True):
        x0, y0 = p0; x1, y1 = p1
        horiz = abs(x1 - x0) >= abs(y1 - y0)
        if horiz:
            a0, a1 = sorted((x0, x1)); fixed = (y0 + y1) / 2.0
        else:
            a0, a1 = sorted((y0, y1)); fixed = (x0 + x1) / 2.0
        ops = sorted([dict(pos=o["pos"], w=o["w"], sill=o.get("sill", 0.0),
                           head=o.get("head", height)) for o in openings], key=lambda o: o["pos"])

        def seg(a_lo, a_hi, z_lo, z_hi, idx):
            if a_hi - a_lo <= 1e-4 or z_hi - z_lo <= 1e-4:
                return
            along = a_hi - a_lo; mid = (a_lo + a_hi) / 2.0; zc = (z_lo + z_hi) / 2.0
            if horiz:
                ctr = (mid, fixed, zc); size = (along, thick, z_hi - z_lo)
            else:
                ctr = (fixed, mid, zc); size = (thick, along, z_hi - z_lo)
            box(f"{path}_{idx}", ctr, size, color, collider=collider)

        c0 = a0; i = 0
        for o in ops:
            o_lo = o["pos"] - o["w"] / 2.0; o_hi = o["pos"] + o["w"] / 2.0
            seg(c0, o_lo, base_z, base_z + height, i); i += 1
            if o["sill"] > 0:
                seg(o_lo, o_hi, base_z, base_z + o["sill"], i); i += 1
            if o["head"] < height:
                seg(o_lo, o_hi, base_z + o["head"], base_z + height, i); i += 1
            c0 = o_hi
        seg(c0, a1, base_z, base_z + height, i)

    def rect_walls(prefix, x0, x1, y0, y1, height, base_z=0.0, thick=0.2,
                   color=C["da_wall"], openings=None, collider=True):
        openings = openings or {}
        wall(f"{prefix}_S", (x0, y0), (x1, y0), height, base_z, thick, openings.get("S", ()), color, collider)
        wall(f"{prefix}_N", (x0, y1), (x1, y1), height, base_z, thick, openings.get("N", ()), color, collider)
        wall(f"{prefix}_W", (x0, y0), (x0, y1), height, base_z, thick, openings.get("W", ()), color, collider)
        wall(f"{prefix}_E", (x1, y0), (x1, y1), height, base_z, thick, openings.get("E", ()), color, collider)

    def slab(path, x0, x1, y0, y1, z, t=0.15, color=C["slab"]):
        box(path, ((x0 + x1) / 2, (y0 + y1) / 2, z), (x1 - x0, y1 - y0, t), color)

    def gable_roof(prefix, x0, x1, y0, y1, eave_z, ridge_z, color, ov=0.4, gable_color=None):
        ym = (y0 + y1) / 2.0
        rx0, rx1 = x0 - ov, x1 + ov
        ny, sy = y1 + ov, y0 - ov
        pts = [(rx0, ym, ridge_z), (rx1, ym, ridge_z), (rx0, ny, eave_z),
               (rx1, ny, eave_z), (rx0, sy, eave_z), (rx1, sy, eave_z)]
        mesh(f"{prefix}_roof", pts, [[0, 1, 3, 2], [0, 4, 5, 1]], color)
        gc = gable_color or color
        mesh(f"{prefix}_gW", [(x0, y0, eave_z), (x0, y1, eave_z), (x0, ym, ridge_z)], [[0, 1, 2]], gc)
        mesh(f"{prefix}_gE", [(x1, y0, eave_z), (x1, y1, eave_z), (x1, ym, ridge_z)], [[0, 1, 2]], gc)

    def gable_frames(prefix, frame_x, y0, y1, eave_z, ridge_z, color):
        """각 프레임선(frame_x)마다 박공 래프터 2개 + 처마/용마루 보."""
        ym = (y0 + y1) / 2.0
        for i, fx in enumerate(frame_x):
            member(f"{prefix}_rafN_{i}", (fx, y1, eave_z), (fx, ym, ridge_z), 0.12, 0.30, color)
            member(f"{prefix}_rafS_{i}", (fx, y0, eave_z), (fx, ym, ridge_z), 0.12, 0.30, color)
        member(f"{prefix}_ridge", (frame_x[0], ym, ridge_z), (frame_x[-1], ym, ridge_z), 0.20, 0.30, color)
        member(f"{prefix}_eaveN", (frame_x[0], y1, eave_z), (frame_x[-1], y1, eave_z), 0.18, 0.30, color)
        member(f"{prefix}_eaveS", (frame_x[0], y0, eave_z), (frame_x[-1], y0, eave_z), 0.18, 0.30, color)
        # 퍼린(중간 1개씩, 양 슬로프)
        for s, yy in (("N", (y1 + ym) / 2), ("S", (y0 + ym) / 2)):
            zz = eave_z + (ridge_z - eave_z) * 0.5
            member(f"{prefix}_purl{s}", (frame_x[0], yy, zz), (frame_x[-1], yy, zz), 0.12, 0.16, color)

    # ============================================================= 지면/도로/펜스 (site)
    F("site", "site")
    GZ = 0.10
    gx0, gx1 = -ROAD_W_WEST - 3.0, SITE["w"] + ROAD_W_EAST + 3.0
    gy0, gy1 = -3.0, SITE["d"] + 3.0
    box("/World/Ground/terrain", ((gx0 + gx1) / 2, (gy0 + gy1) / 2, -GZ / 2),
        (gx1 - gx0, gy1 - gy0, GZ), C["ground"])
    box("/World/Ground/yard", (SITE["w"] / 2, SITE["d"] / 2, 0.005),
        (SITE["w"], SITE["d"], 0.02), C["yard"], collider=False)
    box("/World/Ground/road_west", (-ROAD_W_WEST / 2, SITE["d"] / 2, 0.01),
        (ROAD_W_WEST, SITE["d"], 0.02), C["road"], collider=False)
    box("/World/Ground/road_east", (SITE["w"] + ROAD_W_EAST / 2, SITE["d"] / 2, 0.01),
        (ROAD_W_EAST, SITE["d"], 0.02), C["road"], collider=False)
    fh = 1.8; step = 6.0; posts = []
    x = 0.0
    while x <= SITE["w"] + 1e-6:
        posts += [(x, 0.0), (x, SITE["d"])]; x += step
    y = 0.0
    while y <= SITE["d"] + 1e-6:
        posts += [(0.0, y), (SITE["w"], y)]; y += step
    for pi, (px, py) in enumerate(posts):
        box(f"/World/Fence/p_{pi}", (px, py, fh / 2), (0.12, 0.12, fh), C["fence"], collider=False)

    # ============================================================= 다동 (2층 공장, 상세)
    d = DADONG; wt = d["wall"]
    levels = [("DA-1F", d["f1"], d["f2"], "F1"), ("DA-2F", d["f2"], d["eave"], "F2")]
    door_h = 5.0
    open_1F = {"N": [dict(pos=31.0, w=6.0, head=door_h), dict(pos=52.0, w=6.0, head=door_h),
                     dict(pos=41.5, w=1.2, head=2.4)],
               "S": [dict(pos=41.5, w=6.0, head=door_h)]}
    open_2F = {"N": [dict(pos=x, w=4.0, sill=1.2, head=3.2) for x in (24, 41.5, 59)],
               "S": [dict(pos=x, w=4.0, sill=1.2, head=3.2) for x in (24, 41.5, 59)]}
    for fl, z0, z1, tag in levels:
        h = z1 - z0
        # 외벽
        F(fl, "arch")
        rect_walls(f"/World/Dadong/{tag}/wall", d["x0"], d["x1"], d["y0"], d["y1"], h, z0, wt,
                   C["da_wall"], open_1F if tag == "F1" else open_2F)
        # 기둥(해당 층 구간)
        F(fl, "struct")
        ci = 0
        for gx in d["grid_x"]:
            for gy in d["grid_y"]:
                box(f"/World/Dadong/{tag}/col_{ci}", (gx, gy, (z0 + z1) / 2),
                    (d["col"], d["col"], h), C["column"]); ci += 1
        # 코어(계단/승강기/화장실) 해당 층
        F(fl, "arch")
        for k, cr in enumerate(d["cores"]):
            ctag = "L" if k == 0 else "R"
            side = "E" if k == 0 else "W"
            ow = {"W": (), "E": (), "N": (), "S": ()}
            ow[side] = [dict(pos=(cr["y0"] + cr["y1"]) / 2, w=1.2, head=2.2)]
            rect_walls(f"/World/Dadong/{tag}/core{ctag}", cr["x0"], cr["x1"], cr["y0"], cr["y1"],
                       h, z0, wt, C["core"], ow)
    # 2F 슬래브 + 바닥 철골(거더/보)
    F("DA-2F", "arch")
    slab("/World/Dadong/F2/slab", d["x0"], d["x1"], d["y0"], d["y1"], d["f2"])
    for k, cr in enumerate(d["cores"]):
        slab(f"/World/Dadong/F2/coreSlab{k}", cr["x0"], cr["x1"], cr["y0"], cr["y1"], d["f2"])
    F("DA-2F", "struct")
    gz = d["f2"] - 0.25
    for i, gy in enumerate(d["grid_y"]):                       # 주거더(X방향)
        member(f"/World/Dadong/F2/girder_{i}", (d["x0"], gy, gz), (d["x1"], gy, gz), 0.20, 0.45, C["beam"])
    for i, gx in enumerate(d["grid_x"]):                       # 소보(Y방향)
        member(f"/World/Dadong/F2/beam_{i}", (gx, d["y0"], gz), (gx, d["y1"], gz), 0.15, 0.30, C["beam"])
    # 지붕(클래딩 + 트러스)
    F("DA-roof", "arch")
    gable_roof("/World/Dadong/Roof/clad", d["x0"], d["x1"], d["y0"], d["y1"],
               d["eave"], d["ridge"], C["da_roof"], ov=0.5, gable_color=C["da_wall"])
    box("/World/Dadong/Roof/canopy_N", ((d["x0"] + d["x1"]) / 2, d["y1"] + 1.5, d["eave"] - 0.3),
        (d["x1"] - d["x0"], 3.0, 0.12), C["da_wall"])
    F("DA-roof", "struct")
    gable_frames("/World/Dadong/Roof/fr", d["grid_x"], d["y0"], d["y1"], d["eave"], d["ridge"], C["truss"])

    # ===== 다동 2F(상층) 공정 장비 — dadong_3f_layout 도면 디지털화(67점) =====
    # cat="equip" → 뷰어 '장비' 토글 / USD jm:category="equip". 상층 슬래브 위(z=f2).
    try:
        from dadong_3f_layout import world_boxes as _eq_boxes
        for b in _eq_boxes():
            if b["zone"] == "room":   # 룸(LDI/교반/닦의실)은 영역 표시용 얇은 바닥 패드
                cx, cy, _cz = b["center"]; sx, sy, _sz = b["size"]
                box(f"/World/Dadong/F2/Equip/{b['id']}", (cx, cy, d["f2"] + 0.03),
                    (sx, sy, 0.06), b["color"], collider=False, floor="DA-2F", cat="equip")
            else:                     # 장비는 충돌체(AMR 장애물)
                box(f"/World/Dadong/F2/Equip/{b['id']}", b["center"], b["size"],
                    b["color"], collider=True, floor="DA-2F", cat="equip")
    except Exception as _e:           # 데이터 파일 부재 시에도 월드는 생성
        print("  [warn] dadong_3f_layout 장비 미적용:", _e)

    # ============================================================= 나동 (1층 공장)
    n = NADONG
    F("NA-1F", "arch")
    nopen = {"S": [dict(pos=(n["x0"] + n["x1"]) / 2 - 8, w=5.0, head=4.5),
                   dict(pos=(n["x0"] + n["x1"]) / 2 + 8, w=5.0, head=4.5)],
             "W": [dict(pos=n["y0"] + 8, w=3.5, head=4.0)]}
    rect_walls("/World/Nadong/F1/wall", n["x0"], n["x1"], n["y0"], n["y1"],
               n["eave"] - n["f1"], n["f1"], n["wall"], C["na_wall"], nopen)
    # 사무 칸막이(남서 코너 소규모)
    box("/World/Nadong/F1/office_w", (n["x0"] + 6, (n["y0"] + n["y0"] + 8) / 2 + 0, 1.5),
        (0.15, 8.0, 3.0), C["part_off"])
    box("/World/Nadong/F1/office_w2", ((n["x0"] + n["x0"] + 6) / 2, n["y0"] + 8, 1.5),
        (6.0, 0.15, 3.0), C["part_off"])
    F("NA-1F", "struct")
    nfx = [n["x0"] + i * 7.0 for i in range(int((n["x1"] - n["x0"]) / 7.0) + 1)]
    if nfx[-1] < n["x1"] - 0.5:
        nfx.append(n["x1"])
    for i, gx in enumerate(nfx):                               # 기둥
        for gy in (n["y0"], (n["y0"] + n["y1"]) / 2, n["y1"]):
            box(f"/World/Nadong/F1/col_{i}_{gy:.0f}", (gx, gy, (n["eave"]) / 2),
                (0.3, 0.3, n["eave"]), C["column"])
    F("NA-roof", "arch")
    gable_roof("/World/Nadong/Roof/clad", n["x0"], n["x1"], n["y0"], n["y1"],
               n["eave"], n["ridge"], C["na_roof"], ov=0.4, gable_color=C["na_wall"])
    F("NA-roof", "struct")
    gable_frames("/World/Nadong/Roof/fr", nfx, n["y0"], n["y1"], n["eave"], n["ridge"], C["truss"])

    # ============================================================= 가동 (3층: 식당/사무실/기숙사)
    g = GADONG; gx0, gx1, gy0, gy1 = g["x0"], g["x1"], g["y0"], g["y1"]
    gh = g["fh"]; gw = g["wall"]
    cx = (gx0 + gx1) / 2
    # 계단실(남서, 전층 공통) x[4,7] y[45,49]
    stx0, stx1, sty0, sty1 = gx0, gx0 + 3.0, gy0, gy0 + 4.0
    floors_meta = [("GA-1F", 0, "F1", C["part_din"], "주방/식당"),
                   ("GA-2F", 1, "F2", C["part_off"], "사무실"),
                   ("GA-3F", 2, "F3", C["part_dorm"], "기숙사")]
    for fl, fi, tag, pcol, use in floors_meta:
        z0 = g["floors"][fi]
        F(fl, "arch")
        # 외벽
        if fi == 0:
            gopen = {"S": [dict(pos=cx + 2.5, w=1.5, head=2.4)]}
        else:
            gopen = {"S": [dict(pos=cx, w=6.0, sill=1.0, head=2.4)],
                     "N": [dict(pos=cx, w=6.0, sill=1.0, head=2.4)]}
        rect_walls(f"/World/Gadong/{tag}/wall", gx0, gx1, gy0, gy1, gh, z0, gw, C["ga_wall"], gopen)
        # 바닥 슬래브
        slab(f"/World/Gadong/{tag}/slab", gx0, gx1, gy0, gy1, z0, 0.15, C["slab"])
        # 계단실 벽(홀측 출입문)
        rect_walls(f"/World/Gadong/{tag}/stair", stx0, stx1, sty0, sty1, gh, z0, gw, C["core"],
                   {"E": [dict(pos=(sty0 + sty1) / 2, w=1.0, head=2.1)]})
        # 층별 실내 구획
        zc = z0 + gh / 2
        if fi == 0:   # 1F 주방/식당: 북측 주방 분리
            ky = gy1 - 5.0
            box(f"/World/Gadong/{tag}/kitchen_w", (cx, ky, zc), (gx1 - gx0, 0.15, gh), C["kitchen"])
            box(f"/World/Gadong/{tag}/kitchen_door", (cx, ky, z0 + 2.5), (1.4, 0.18, gh - 2.5), C["ga_wall"])
        elif fi == 1:   # 2F 사무실: 중복도 + 동측 사무실 2칸
            cory = gy0 + 9.0
            box(f"/World/Gadong/{tag}/corr_w", (cx + 1.0, (gy0 + 4 + gy1) / 2, zc),
                (0.15, gy1 - (gy0 + 4), gh), C["part_off"])
            for j, yy in enumerate((gy0 + 7.5, gy0 + 12.5)):  # 사무실 칸막이
                box(f"/World/Gadong/{tag}/off_{j}", ((cx + 1 + gx1) / 2, yy, zc),
                    (gx1 - (cx + 1), 0.15, gh), C["part_off"])
        else:           # 3F 기숙사: 중복도 + 양측 침실 각 3칸
            box(f"/World/Gadong/{tag}/corr_wW", (cx - 1.2, (gy0 + 4 + gy1) / 2, zc),
                (0.15, gy1 - (gy0 + 4), gh), C["part_dorm"])
            box(f"/World/Gadong/{tag}/corr_wE", (cx + 1.2, (gy0 + 4 + gy1) / 2, zc),
                (0.15, gy1 - (gy0 + 4), gh), C["part_dorm"])
            for j in range(3):
                yy = gy0 + 6.0 + j * 4.0
                box(f"/World/Gadong/{tag}/roomW_{j}", ((gx0 + cx - 1.2) / 2, yy, zc),
                    (cx - 1.2 - gx0, 0.15, gh), C["part_dorm"])
                box(f"/World/Gadong/{tag}/roomE_{j}", ((cx + 1.2 + gx1) / 2, yy, zc),
                    (gx1 - (cx + 1.2), 0.15, gh), C["part_dorm"])
        # 층 천장(상부 바닥) 철골 보
        F(fl, "struct")
        for j in range(4):
            yy = gy0 + 2.0 + j * 4.0
            member(f"/World/Gadong/{tag}/beam_{j}", (gx0, yy, z0 + gh - 0.22), (gx1, yy, z0 + gh - 0.22),
                   0.12, 0.30, C["beam"])
    # 옥상 + 파라펫
    F("GA-roof", "arch")
    slab("/World/Gadong/Roof/slab", gx0, gx1, gy0, gy1, g["roof"], 0.15, C["ga_roof"])
    rect_walls("/World/Gadong/Roof/parapet", gx0, gx1, gy0, gy1, g["parapet"], g["roof"], 0.15, C["ga_wall"])

    return parts


if __name__ == "__main__":
    p = build_scene()
    from collections import Counter
    print("parts:", len(p), "boxes:", sum(x["type"] == "box" for x in p),
          "meshes:", sum(x["type"] == "mesh" for x in p))
    print("by floor:", dict(Counter(x["floor"] for x in p)))
    print("by cat  :", dict(Counter(x["cat"] for x in p)))
