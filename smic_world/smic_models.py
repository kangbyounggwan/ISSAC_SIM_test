#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
smic_models.py — SMIC World 설비 클래스별 절차적 모델(순수 pxr, Kit 불필요).

JM(model_equipment.py) / 서한(equipment_models.py) 패턴을 KETI 컨셉 셀에 맞게 옮겼다.
색 박스 대신 클래스별로 알아볼 수 있는 형상(갠트리·경사 슈트·C프레임 프레스·AMR 등)을
프리미티브로 조립한다.

로컬 프레임 규약: +X=폭, +Y=깊이, z=0=바닥(스테이션 Xform 이 월드 배치를 담당).
**정면 = 로컬 -Y**. 치수는 전부 smic_layout.py 에서 주입받는다(여기에 상수 금지).

콜리전: 지지면/구조체에만 CollisionAPI 를 건다(장식 바·메시봉은 시각 전용).
JM 교훈 — 지지면 콜라이더를 빼먹으면 판재가 관통해 떨어진다.
"""
import math
from pxr import UsdGeom, UsdPhysics, Gf, Sdf

from smic_layout import (C_STEEL, C_DARK, C_WHITE, C_METAL, C_GLASS, C_SIGN,
                         C_ORANGE, C_YELLOW, C_AMR, BOX, PLATE)

RED = (0.82, 0.20, 0.18); AMBER = (0.95, 0.66, 0.15); GREEN = (0.20, 0.70, 0.32)


# ------------------------------------------------------------------ 프리미티브
def box(stage, path, c, s, color, coll=False, rot_x=0.0):
    """중심 c=(x,y,z), 크기 s=(w,d,h) 축정렬 박스. rot_x 는 로컬 X축 회전(도)."""
    q = UsdGeom.Cube.Define(stage, path)
    q.CreateSizeAttr(1.0)
    q.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
    q.AddTranslateOp().Set(Gf.Vec3d(*c))
    if abs(rot_x) > 1e-9:
        q.AddRotateXOp().Set(rot_x)
    q.AddScaleOp().Set(Gf.Vec3f(*[max(float(v), 1e-3) for v in s]))
    q.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if coll:
        UsdPhysics.CollisionAPI.Apply(q.GetPrim())
    return q


def cyl(stage, path, c, r, h, axis, color, coll=False):
    q = UsdGeom.Cylinder.Define(stage, path)
    q.CreateRadiusAttr(r); q.CreateHeightAttr(h); q.CreateAxisAttr(axis)
    ext = {"Z": [(-r, -r, -h / 2), (r, r, h / 2)],
           "Y": [(-r, -h / 2, -r), (r, h / 2, r)],
           "X": [(-h / 2, -r, -r), (h / 2, r, r)]}[axis]
    q.CreateExtentAttr(ext)
    q.AddTranslateOp().Set(Gf.Vec3d(*c))
    q.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if coll:
        UsdPhysics.CollisionAPI.Apply(q.GetPrim())
    return q


def sign_plate(stage, path, c, s, text, color=C_SIGN):
    """개념도의 파란 라벨판. 글자는 렌더 폰트 없이 못 그리므로 customData 로 보존."""
    q = box(stage, path, c, s, color)
    q.GetPrim().SetCustomDataByKey("text", text)
    return q


def signal_tower(stage, pfx, x, y, ztop):
    cyl(stage, pfx + "/sig_pole", (x, y, ztop + 0.14), 0.028, 0.28, "Z", C_DARK)
    for i, col in enumerate((GREEN, AMBER, RED)):
        cyl(stage, pfx + "/sig_%d" % i, (x, y, ztop + 0.32 + i * 0.10), 0.055, 0.09, "Z", col)


def _legs(stage, pfx, W, D, h, inset=0.06, t=0.05, color=C_STEEL, coll=True):
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        box(stage, pfx + "/leg_%d" % i,
            (sx * (W / 2 - inset), sy * (D / 2 - inset), h / 2), (t, t, h), color, coll=coll)


# ------------------------------------------------------------------ 1. 레이저 커팅기
def build_laser_cutter(stage, pfx, W, D, H, spec):
    """개방형 갠트리 평판 레이저. 정면(-Y)에 기계명 패널 + 좌측 전면에 제어 콘솔."""
    body_t, body_h = 0.22, 0.62
    base_h = 0.34
    box(stage, pfx + "/base", (0, 0, base_h / 2), (W, D, base_h), C_DARK, coll=True)
    top = base_h + body_h                                     # 몸체 상단 = 0.96
    for nm, c, s in (
        ("body_front", (0, -D / 2 + body_t / 2, base_h + body_h / 2), (W, body_t, body_h)),
        ("body_back",  (0,  D / 2 - body_t / 2, base_h + body_h / 2), (W, body_t, body_h)),
        ("body_left",  (-W / 2 + body_t / 2, 0, base_h + body_h / 2), (body_t, D, body_h)),
        ("body_right", (W / 2 - body_t / 2, 0, base_h + body_h / 2), (body_t, D, body_h)),
    ):
        box(stage, pfx + "/" + nm, c, s, C_WHITE, coll=True)

    # 슬랫 베드(가공 테이블) — 몸체 안쪽으로 함몰
    bw, bd = W - 2 * body_t - 0.10, D - 2 * body_t - 0.10
    box(stage, pfx + "/bed", (0, 0, base_h + 0.06), (bw, bd, 0.12), (0.13, 0.14, 0.16), coll=True)
    n_slat = max(6, int(bw / 0.34))
    for i in range(n_slat):
        sx = -bw / 2 + (i + 0.5) * bw / n_slat
        box(stage, pfx + "/slat_%d" % i, (sx, 0, base_h + 0.16), (0.02, bd * 0.96, 0.09), C_METAL)
    # 배기 포트(개념도의 원형 개구부)
    cyl(stage, pfx + "/exhaust", (-bw / 2 + 0.55, bd / 2 - 0.55, base_h + 0.14),
        0.28, 0.06, "Z", (0.10, 0.11, 0.12))

    # 좌우 가이드 레일 + 갠트리 브리지 + 커팅 헤드
    for sx, nm in ((-1, "railL"), (1, "railR")):
        box(stage, pfx + "/" + nm, (sx * (W / 2 - body_t / 2), 0, top + 0.05),
            (body_t * 0.9, D * 0.96, 0.10), C_METAL, coll=True)
    gy = D * 0.18
    box(stage, pfx + "/gantry", (0, gy, top + 0.34), (W - 0.10, 0.34, 0.38), C_WHITE)
    box(stage, pfx + "/carriage", (W * 0.10, gy, top + 0.30), (0.42, 0.42, 0.50), C_DARK)
    cyl(stage, pfx + "/head", (W * 0.10, gy, top - 0.10), 0.055, 0.26, "Z", C_METAL)

    # 전면 기계명 패널
    sign_plate(stage, pfx + "/nameplate",
               (0, -D / 2 - 0.012, base_h + body_h * 0.45), (W * 0.42, 0.02, 0.15),
               spec.get("label", "LASER CUTTING MACHINE"), color=(0.96, 0.96, 0.96))

    # 좌측-전면 제어 콘솔 (개념도: 키패드가 달린 별치 콘솔)
    cx, cy = -W / 2 - 0.32, -D / 2 + 0.55
    box(stage, pfx + "/console_post", (cx, cy, 0.48), (0.14, 0.14, 0.96), C_DARK, coll=True)
    box(stage, pfx + "/console_head", (cx, cy - 0.10, 1.10), (0.48, 0.34, 0.26), C_DARK, coll=True)
    box(stage, pfx + "/console_screen", (cx, cy - 0.27, 1.12), (0.34, 0.02, 0.18), (0.14, 0.30, 0.52))
    box(stage, pfx + "/console_keys", (cx, cy - 0.10, 1.24), (0.40, 0.26, 0.02), (0.55, 0.56, 0.58))
    signal_tower(stage, pfx, -W / 2 + 0.35, D / 2 - 0.35, top + 0.10)


# ------------------------------------------------------------------ 2. 중력 슬라이드
def build_gravity_slide(stage, pfx, W, Ylen, drop, spec):
    """2라인 중력식 슈트. 로컬 원점 = 전면-하단(픽 지점). +Y 로 갈수록 높아진다."""
    lanes = int(spec.get("lanes", 2))
    lane_w = float(spec.get("lane_w", 0.62))
    z0 = float(spec.get("deck_z", 1.05))          # 랜딩 데크 top
    land = float(spec.get("landing_len", 0.45))
    run = Ylen - land
    theta = math.degrees(math.atan2(drop, run))
    seg = math.hypot(run, drop)
    deck_t = 0.03

    lane_x = [(-(lanes - 1) / 2.0 + i) * (lane_w + 0.06) for i in range(lanes)]

    for i, lx in enumerate(lane_x):
        # 수평 랜딩(픽 위치) — 판재가 여기서 멈춘다
        box(stage, pfx + "/land_%d" % i, (lx, land / 2, z0 - deck_t / 2),
            (lane_w, land, deck_t), (0.72, 0.74, 0.76), coll=True)
        # 경사 데크
        box(stage, pfx + "/chute_%d" % i, (lx, land + run / 2, z0 - deck_t / 2 + drop / 2),
            (lane_w, seg, deck_t), (0.72, 0.74, 0.76), coll=True, rot_x=theta)
        # 레인 측면 가이드
        for sx in (-1, 1):
            gx = lx + sx * (lane_w / 2 - 0.015)
            box(stage, pfx + "/guide_%d%s" % (i, "L" if sx < 0 else "R"),
                (gx, land / 2, z0 + 0.04), (0.03, land, 0.10), C_STEEL, coll=True)
            box(stage, pfx + "/chguide_%d%s" % (i, "L" if sx < 0 else "R"),
                (gx, land + run / 2, z0 + 0.04 + drop / 2), (0.03, seg, 0.10),
                C_STEEL, coll=True, rot_x=theta)

    # 전면 스토퍼(판재 이탈 방지)
    box(stage, pfx + "/stopper", (0, -0.02, z0 + 0.04), (W, 0.05, 0.10), C_DARK, coll=True)

    # 프레임: 전/중/후 다리쌍 + 사다리형 가로대
    def leg(nm, y, h):
        for sx in (-1, 1):
            box(stage, pfx + "/%s_%s" % (nm, "L" if sx < 0 else "R"),
                (sx * (W / 2 - 0.05), y, h / 2), (0.07, 0.07, h), C_STEEL, coll=True)

    leg("legF", 0.10, z0 - deck_t)
    leg("legM", land + run * 0.5, z0 - deck_t + drop * 0.5)
    leg("legB", Ylen - 0.12, z0 - deck_t + drop)
    n_cross = max(3, int(seg / 0.55))
    for k in range(n_cross):
        t = (k + 0.5) / n_cross
        box(stage, pfx + "/cross_%d" % k,
            (0, land + run * t, z0 - deck_t - 0.05 + drop * t), (W, 0.05, 0.05), C_STEEL)
    # 후단 상부 프레임(개념도의 높은 백 프레임)
    for sx in (-1, 1):
        box(stage, pfx + "/upright_%s" % ("L" if sx < 0 else "R"),
            (sx * (W / 2 - 0.05), Ylen - 0.12, z0 + drop + 0.20),
            (0.07, 0.07, 0.55), C_STEEL)
    box(stage, pfx + "/uptop", (0, Ylen - 0.12, z0 + drop + 0.46), (W, 0.07, 0.07), C_STEEL)


# ------------------------------------------------------------------ 2b. 겐트리 로더
def build_gantry_loader(stage, pfx, W, D, H, spec):
    """문형(portal) 오버헤드 겐트리. 로컬 +X = 스팬축, H = 브리지 하단 높이.

    다리 2개(양 끝) + 박스거더 브리지 + X 주행 캐리지 + 텔레스코픽 Z 마스트 +
    진공 리프터 헤드. 레이저/슬라이드를 **타넘기** 때문에 다리 외에는 바닥에
    닿는 부분이 없다 (smic_layout 의 ground_boxes 참조).
    """
    cxl = float(spec.get("carriage_x", 0.0))       # 캐리지 로컬 X
    head_z = float(spec.get("head_z", H - 1.2))    # 진공 리프터 하면 z

    # --- 다리 2개 (베이스 플레이트 + 박스 컬럼 + 캡 + 헌치) ------------------
    for sx in (-1, 1):
        tag = "L" if sx < 0 else "R"
        lx = sx * W / 2.0
        box(stage, pfx + "/base_%s" % tag, (lx, 0, 0.04), (0.70, 0.90, 0.08),
            C_DARK, coll=True)
        box(stage, pfx + "/col_%s" % tag, (lx, 0, 0.08 + (H - 0.14) / 2),
            (0.32, 0.42, H - 0.14), C_STEEL, coll=True)
        box(stage, pfx + "/cap_%s" % tag, (lx, 0, H - 0.03), (0.50, 0.60, 0.06),
            C_METAL, coll=True)
        # 상부 헌치(브리지 접합부 보강)
        box(stage, pfx + "/haunch_%s" % tag, (lx - sx * 0.42, 0, H - 0.20),
            (0.60, 0.34, 0.28), C_STEEL)
        # 안전 표시(다리 하부 옐로 밴드)
        box(stage, pfx + "/band_%s" % tag, (lx, 0, 0.55), (0.34, 0.44, 0.30), C_YELLOW)

    # --- 브리지 박스거더 + 캐리지 주행 레일 ----------------------------------
    box(stage, pfx + "/girder", (0, 0, H + 0.30), (W + 0.60, D, 0.55), C_WHITE, coll=True)
    box(stage, pfx + "/rail", (0, 0, H + 0.02), (W + 0.40, D * 0.42, 0.10), C_METAL, coll=True)
    # 케이블 드래그체인(전면)
    box(stage, pfx + "/dragchain", (0, -D / 2 - 0.07, H + 0.44),
        (W * 0.82, 0.10, 0.12), C_DARK)
    # 점검 통로 핸드레일(후면)
    hr_z = H + 0.58
    box(stage, pfx + "/hrail", (0, D / 2 + 0.04, hr_z + 0.52), (W, 0.04, 0.04), C_STEEL)
    n_post = max(2, int(W / 1.6))
    for i in range(n_post + 1):
        px = -W / 2 + i * (W / n_post)
        box(stage, pfx + "/hpost_%d" % i, (px, D / 2 + 0.04, hr_z + 0.26),
            (0.04, 0.04, 0.52), C_STEEL)

    # --- 캐리지 (X 주행) ------------------------------------------------------
    box(stage, pfx + "/carriage", (cxl, 0, H - 0.20), (0.85, D * 0.92, 0.34), C_DARK)
    for sx in (-1, 1):
        box(stage, pfx + "/truck_%s" % ("A" if sx < 0 else "B"),
            (cxl + sx * 0.32, 0, H + 0.02), (0.18, D * 0.5, 0.12), C_METAL)

    # --- 텔레스코픽 Z 마스트 --------------------------------------------------
    top = H - 0.37
    bot = head_z + 0.10
    span = max(top - bot, 0.10)
    box(stage, pfx + "/mast_1", (cxl, 0, top - span * 0.28), (0.24, 0.24, span * 0.56),
        C_METAL)
    box(stage, pfx + "/mast_2", (cxl, 0, bot + span * 0.28), (0.17, 0.17, span * 0.56),
        C_STEEL)

    # --- 진공 리프터 헤드 ------------------------------------------------------
    box(stage, pfx + "/head", (cxl, 0, head_z + 0.06), (0.78, 0.58, 0.12), C_METAL, coll=True)
    for i in range(3):
        for j in range(2):
            hx = cxl - 0.26 + i * 0.26
            hy = -0.14 + j * 0.28
            cyl(stage, pfx + "/cup_%d%d" % (i, j), (hx, hy, head_z - 0.03),
                0.06, 0.06, "Z", C_DARK)

    signal_tower(stage, pfx, -W / 2, D / 2 + 0.10, H + 0.58)


# ------------------------------------------------------------------ 3. 박스 테이블
def build_box_table(stage, pfx, W, D, H, spec):
    """분류 테이블 / Press 완료 적재대. 상판 + 하단 선반 + 정면 파란 라벨판."""
    box(stage, pfx + "/top", (0, 0, H - 0.02), (W, D, 0.04), C_METAL, coll=True)
    box(stage, pfx + "/shelf", (0, 0, 0.28), (W - 0.10, D - 0.06, 0.03), C_METAL, coll=True)
    _legs(stage, pfx, W, D, H, inset=0.05, t=0.05)
    for sy, nm in ((-1, "railF"), (1, "railB")):
        box(stage, pfx + "/" + nm, (0, sy * (D / 2 - 0.03), H - 0.14),
            (W - 0.10, 0.03, 0.05), C_STEEL)

    n = int(spec.get("slots", 0)); p = float(spec.get("slot_pitch", 0.80))
    labels = spec.get("slot_labels") or []
    for i in range(n):
        sx = -(n - 1) * p / 2.0 + i * p
        txt = labels[i] if i < len(labels) else ""
        sign_plate(stage, pfx + "/slot_sign_%d" % i,
                   (sx, -D / 2 - 0.012, H - 0.11), (p * 0.86, 0.02, 0.13), txt)


# ------------------------------------------------------------------ 4. 빈 박스 랙
def build_box_rack(stage, pfx, W, D, H, spec):
    shelves = spec.get("shelves") or [0.30, 0.85, 1.40]
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        box(stage, pfx + "/post_%d" % i,
            (sx * (W / 2 - 0.04), sy * (D / 2 - 0.04), H / 2), (0.06, 0.06, H), C_STEEL, coll=True)
    for i, z in enumerate(shelves):
        box(stage, pfx + "/shelf_%d" % i, (0, 0, z), (W - 0.06, D - 0.06, 0.035),
            C_METAL, coll=True)
    for sy in (-1, 1):
        box(stage, pfx + "/brace_%s" % ("F" if sy < 0 else "B"),
            (0, sy * (D / 2 - 0.04), H - 0.05), (W - 0.06, 0.04, 0.04), C_STEEL)


# ------------------------------------------------------------------ 5. PRESS M/C
def build_press(stage, pfx, W, D, H, spec):
    """갭(C) 프레임 프레스. 정면(-Y) 개방, 볼스터 위 다이 상면 = spec['die_z']."""
    die_z = float(spec.get("die_z", 1.05))
    dy = float(spec.get("die_offset_y", -0.10))    # 다이를 베드 전면 쪽으로 당김
    base_h = die_z - 0.30
    box(stage, pfx + "/base", (0, 0, base_h / 2), (W, D, base_h), C_DARK, coll=True)
    box(stage, pfx + "/bolster", (0, dy, base_h + 0.08), (W * 0.70, D * 0.60, 0.16),
        C_METAL, coll=True)
    box(stage, pfx + "/die", (0, dy, base_h + 0.16 + 0.07), (W * 0.46, D * 0.42, 0.14),
        C_ORANGE, coll=True)                                   # 다이 상면 = die_z

    col_d = D * 0.32
    box(stage, pfx + "/column", (0, D / 2 - col_d / 2, base_h + (H - base_h) / 2),
        (W, col_d, H - base_h), C_WHITE, coll=True)
    for sx in (-1, 1):
        box(stage, pfx + "/side_%s" % ("L" if sx < 0 else "R"),
            (sx * (W / 2 - 0.10), 0, base_h + (H - base_h) * 0.55),
            (0.20, D * 0.72, (H - base_h) * 0.92), C_WHITE, coll=True)
    crown_h = H * 0.20
    box(stage, pfx + "/crown", (0, 0, H - crown_h / 2), (W, D * 0.92, crown_h), C_WHITE, coll=True)
    box(stage, pfx + "/ram", (0, dy, die_z + 0.55), (W * 0.44, D * 0.40, 0.55), C_METAL)
    box(stage, pfx + "/guard", (0, -D / 2 + 0.06, die_z + 0.70), (W * 0.78, 0.03, 0.85), C_GLASS)

    # 우측 제어반
    box(stage, pfx + "/ctrl", (W / 2 + 0.28, -D * 0.15, 0.85), (0.45, 0.60, 1.70),
        C_METAL, coll=True)
    box(stage, pfx + "/hmi", (W / 2 + 0.28, -D * 0.15 - 0.31, 1.28), (0.32, 0.03, 0.26),
        (0.12, 0.20, 0.45))
    sign_plate(stage, pfx + "/nameplate", (0, -D / 2 - 0.012, H - crown_h * 0.55),
               (W * 0.55, 0.02, 0.18), spec.get("label", "PRESS M/C"), color=(0.96, 0.96, 0.96))
    signal_tower(stage, pfx, -W / 2 + 0.30, 0.0, H)


# ------------------------------------------------------------------ 6. AMR Port
def build_amr_port(stage, pfx, W, D, H, spec):
    """AMR 도킹 포트: 상단 데크(박스 1개) + 정면 파란 사인. 하부는 개방(AMR 진입)."""
    box(stage, pfx + "/deck", (0, 0, H - 0.025), (W, D, 0.05), C_METAL, coll=True)
    _legs(stage, pfx, W, D, H, inset=0.05, t=0.05)
    box(stage, pfx + "/brace", (0, 0, 0.14), (W - 0.10, 0.04, 0.04), C_STEEL)
    for sx in (-1, 1):
        box(stage, pfx + "/signpost_%s" % ("L" if sx < 0 else "R"),
            (sx * (W / 2 - 0.10), -D / 2 + 0.05, H + 0.18), (0.04, 0.04, 0.36), C_STEEL)
    sign_plate(stage, pfx + "/sign", (0, -D / 2 + 0.05, H + 0.30),
               (W * 0.92, 0.02, 0.24), spec.get("sign", "AMR Port"))


# ------------------------------------------------------------------ 7. AMR
def build_amr(stage, pfx, W, D, H, spec):
    wr = 0.09
    box(stage, pfx + "/body", (0, 0, wr + H / 2), (W, D, H), C_AMR, coll=True)
    box(stage, pfx + "/deck", (0, 0, wr + H + 0.012), (W * 0.96, D * 0.96, 0.025),
        (0.72, 0.74, 0.76), coll=True)
    box(stage, pfx + "/bumper", (0, 0, wr + 0.05), (W * 1.01, D * 1.01, 0.06), C_DARK)
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        cyl(stage, pfx + "/wheel_%d" % i,
            (sx * (W / 2 - 0.17), sy * (D / 2 - 0.02), wr), wr, 0.06, "Y", C_DARK)
    cyl(stage, pfx + "/lidar", (0, -D / 2 + 0.10, wr + H + 0.06), 0.055, 0.07, "Z", C_DARK)


# ------------------------------------------------------------------ 8. 안전 펜스
def build_fence_panel(stage, pfx, length, h):
    """메시 펜스 1패널. 로컬 +X = 패널 길이축. 포스트/레일만 콜리전."""
    for sx in (-1, 1):
        box(stage, pfx + "/post_%s" % ("A" if sx < 0 else "B"),
            (sx * (length / 2 - 0.03), 0, h / 2), (0.06, 0.06, h), C_STEEL, coll=True)
    for k, z in ((0, 0.10), (1, h / 2), (2, h - 0.06)):
        box(stage, pfx + "/rail_%d" % k, (0, 0, z), (length, 0.05, 0.05), C_STEEL,
            coll=(k == 1))
    n = max(4, int(length / 0.20))
    for i in range(n):
        bx = -length / 2 + (i + 0.5) * length / n
        box(stage, pfx + "/mesh_%d" % i, (bx, 0, h / 2), (0.012, 0.012, h - 0.16), C_STEEL)


# ------------------------------------------------------------------ 9. 컨테이너(동적)
def build_container(stage, path, center, yaw=0.0, color=None, dynamic=True, mass=None):
    """녹색 물류 컨테이너 — **상부 개방(뚜껑 없음)** 박스.

    바닥판 1 + 벽 4 로 속이 빈 형상을 만들고, 상단은 개구부를 두르는 **4변 림 프레임**
    으로만 마감한다(판 하나로 덮으면 닫힌 상자가 되어 물건을 넣지 못한다).
    림은 바깥으로 살짝 돌출 — 실물 KLT 의 보강 리브이자 그리퍼가 무는 자리라
    콜리전을 준다. 전체가 단일 강체.
    center = 컨테이너 **바닥면 중심** 월드 좌표."""
    w, d, h, t = BOX["w"], BOX["d"], BOX["h"], BOX["t"]
    col = color or BOX["color"]
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], center[2] + h / 2.0))
    if abs(yaw) > 1e-9:
        xf.AddRotateZOp().Set(yaw)
    prim = xf.GetPrim()
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass if mass is not None else BOX["mass"]))
        prim.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
    # 로컬 원점 = 박스 중심
    parts = [
        ("bottom", (0, 0, -h / 2 + t / 2), (w, d, t)),
        ("wall_f", (0, -d / 2 + t / 2, 0), (w, t, h)),
        ("wall_b", (0,  d / 2 - t / 2, 0), (w, t, h)),
        ("wall_l", (-w / 2 + t / 2, 0, 0), (t, d, h)),
        ("wall_r", (w / 2 - t / 2, 0, 0), (t, d, h)),
    ]
    for nm, c, s in parts:
        box(stage, path + "/" + nm, c, s, col, coll=True)

    # 상단 림: 개구부 4변만 (상판 X). 바깥으로 rim_out 만큼 돌출.
    rim_c = (col[0] * 0.78, col[1] * 0.78, col[2] * 0.78)
    rim_out, rim_t, rim_h = 0.014, t + 0.024, 0.026
    rz = h / 2 - rim_h / 2
    rim = [
        ("rim_f", (0, -d / 2 + t / 2, rz), (w + 2 * rim_out, rim_t, rim_h)),
        ("rim_b", (0,  d / 2 - t / 2, rz), (w + 2 * rim_out, rim_t, rim_h)),
        ("rim_l", (-w / 2 + t / 2, 0, rz), (rim_t, d + 2 * rim_out, rim_h)),
        ("rim_r", (w / 2 - t / 2, 0, rz), (rim_t, d + 2 * rim_out, rim_h)),
    ]
    for nm, c, s in rim:
        box(stage, path + "/" + nm, c, s, rim_c, coll=True)
    return prim


# ------------------------------------------------------------------ 10. 판재(동적)
def build_plate(stage, path, center, yaw=0.0, pitch=0.0, dynamic=True):
    """커팅 블랭크. center = 판재 **중심** 월드. pitch = 로컬 X축 경사(도, 슈트용)."""
    q = UsdGeom.Cube.Define(stage, path)
    q.CreateSizeAttr(1.0)
    q.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
    q.AddTranslateOp().Set(Gf.Vec3d(*center))
    if abs(yaw) > 1e-9:
        q.AddRotateZOp().Set(yaw)
    if abs(pitch) > 1e-9:
        q.AddRotateXOp().Set(pitch)
    q.AddScaleOp().Set(Gf.Vec3f(PLATE["w"], PLATE["d"], PLATE["t"]))
    q.CreateDisplayColorAttr([Gf.Vec3f(*PLATE["color"])])
    prim = q.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(PLATE["mass"]))
        prim.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.005)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    return prim


DISPATCH = {
    "laser_cutter": build_laser_cutter,
    "gravity_slide": build_gravity_slide,
    "gantry_loader": build_gantry_loader,
    "box_table": build_box_table,
    "box_rack": build_box_rack,
    "press": build_press,
    "amr_port": build_amr_port,
    "amr": build_amr,
}


def build_station(stage, path, spec):
    """스테이션 Xform(월드 배치) + 클래스별 상세 모델(로컬)."""
    cx, cy = spec["center"]
    W, D, H = spec["size"]
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, 0.0))
    yaw = float(spec.get("yaw", 0.0))
    if abs(yaw) > 1e-9:
        xf.AddRotateZOp().Set(yaw)
    prim = xf.GetPrim()
    for k in ("id", "name", "cls"):
        prim.SetCustomDataByKey(k, spec[k])
    prim.SetCustomDataByKey("size_wdh", Gf.Vec3d(*[float(v) for v in spec["size"]]))
    fn = DISPATCH.get(spec["cls"])
    if fn is None:
        raise KeyError("no model builder for cls=%r (station %s)" % (spec["cls"], spec["id"]))
    fn(stage, path, W, D, H, spec)
    return prim
