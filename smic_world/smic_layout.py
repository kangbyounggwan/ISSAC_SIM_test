#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
smic_layout.py — SMIC World 레이아웃 **단일 진실원(single source of truth)**.

KETI 컨셉(안) 개념도에는 실측 치수가 없다. 따라서 모든 수치는
  (1) 개념도의 상대 비율(휴머노이드 신장 1.75 m 를 스케일 바로 사용)
  (2) 해당 설비 클래스의 표준 실물 치수(레이저 3015, KLT 600×400 컨테이너 등)
두 가지를 교차시켜 산출한 **가정값**이다. 실측이 들어오면 이 파일의 숫자만
고치면 월드 전체가 따라간다(모델/빌드 스크립트에는 상수를 두지 않는다).

가정의 근거는 SMIC_WORLD_DESIGN.md 의 치수표에 항목별로 적어 두었다.

좌표 규약 (JM 월드와 동일):
  Z-up, meters. 바닥 슬래브 top = z 0.0.
  +X = 개념도 왼쪽→오른쪽 (레이저 → 슬라이드 → 프레스 → AMR)
  +Y = 개념도 앞쪽(관찰자)→뒤쪽(창문 벽)
  원점 (0,0) = 베이 좌측-전면 안쪽 코너.
  각 스테이션은 자기 로컬 프레임에서 +X=폭, +Y=깊이, z=0=바닥, **정면 = 로컬 -Y**.
  yaw(도)는 월드 Z축 회전. 개념도의 설비는 전부 정면이 -Y(통로 쪽)라 yaw=0.
"""

# ============================================================== 공통 상수
FLOOR_Z = 0.0            # 슬래브 top. 모든 설비/로봇 발바닥 기준면
WALL_H = 6.0             # 베이 내부 유효고 (개념도 창문 상단 + 여유)
BAY = {                  # 건물 베이 (내부 치수)
    "x0": 0.0, "x1": 22.0,
    "y0": 0.0, "y1": 15.0,
    "h": WALL_H,
}
SLAB_MARGIN = 1.5        # 슬래브가 벽 바깥으로 더 나가는 폭

# 표준 물류 컨테이너 (KLT/유로박스 600×400×320) — 개념도의 녹색 박스
BOX = {"w": 0.60, "d": 0.40, "h": 0.32, "t": 0.012, "mass": 1.8,
       "color": (0.22, 0.32, 0.24)}

# 커팅 블랭크(타공 판재). 개념도의 슬라이드/박스 안 검은 판재.
# 500×350×2.0t 강판 = 2.7 kg 이나 타공으로 실질 ~55% 제거 → 1.2 kg 로 가정.
PLATE = {"w": 0.50, "d": 0.35, "t": 0.002, "mass": 1.2,
         "color": (0.15, 0.16, 0.18)}

# ============================================================== 팔레트
C_CONCRETE = (0.62, 0.63, 0.64)
C_WALL     = (0.78, 0.79, 0.80)
C_GLASS    = (0.58, 0.72, 0.86)
C_STEEL    = (0.55, 0.57, 0.60)
C_DARK     = (0.17, 0.18, 0.20)
C_WHITE    = (0.90, 0.90, 0.91)
C_METAL    = (0.68, 0.70, 0.73)
C_YELLOW   = (0.95, 0.80, 0.15)
C_ORANGE   = (0.92, 0.55, 0.10)
C_SIGN     = (0.09, 0.20, 0.52)   # 개념도의 파란 라벨판
C_AMR      = (0.93, 0.94, 0.95)

# ============================================================== 스테이션
# center = (x, y) 월드, size = (W, D, H) 로컬(폭X, 깊이Y, 높이Z), yaw = 도.
# cls    = smic_models.DISPATCH 키.
# ground_boxes = 바닥에 **실제로 닿는** 부분의 로컬 풋프린트 [(dx,dy,w,d), ...].
#   생략하면 전체 bbox 를 풋프린트로 본다. 겐트리처럼 다른 설비를 **타넘는**
#   구조물은 이걸 줘야 침범/클리어런스 검사가 올바르게 돈다.
STATIONS = [
    # --- 레이저 커팅기 -------------------------------------------------------
    # 3015급 평판 파이버 레이저(가공영역 3.0×1.5m)의 개방형 갠트리 프레임.
    # 개념도 비율상 로봇 신장의 ~3배 폭 → W 5.2 m 로 가정.
    dict(id="LASER_CUT", cls="laser_cutter", name="레이저 커팅기",
         center=(3.4, 9.0), size=(5.2, 4.4, 1.80), yaw=0.0,
         label="LASER CUTTING MACHINE"),

    # --- 겐트리 로더 (레이저 베드 → 슬라이드 상단) -------------------------------
    # 문형(portal) 오버헤드 겐트리. 다리는 레이저(x 0.8~6.0)와 슬라이드(x 7.83~9.38)
    # 풋프린트를 피해 **양 끝에만** 선다 → 다리 x = 0.40 / 9.90, 스팬 9.50 m.
    # y = 10.40 은 레이저 베드 후방(y 7.16~10.84)과 슬라이드 고측(deck z 2.02)을
    # 동시에 덮는 유일한 라인이다. 브리지 하단 z = 3.90 (설비 최고점 대비 여유 1.1 m).
    dict(id="GANTRY", cls="gantry_loader", name="겐트리 로더(레이저→슬라이드)",
         center=(5.15, 10.40), size=(9.50, 0.60, 3.90), yaw=0.0,
         carriage_x=2.05,        # 로컬 X (월드 7.20 = 레이저와 슬라이드 사이)
         head_z=2.70,            # 진공 리프터 하면 z
         ground_boxes=[(-4.75, 0.0, 0.70, 0.90), (4.75, 0.0, 0.70, 0.90)]),

    # --- 커팅 제품 슬라이드 (2라인, 중력식) ------------------------------------
    # 로컬 원점 = **전면-하단 끝**(픽 지점). +Y 로 갈수록 높아진다.
    # size = (전체폭, 수평투영길이, 낙차). 경사 = atan(1.25/3.80) = 18.2°.
    dict(id="SLIDE", cls="gravity_slide", name="커팅 제품 슬라이드(2라인)",
         center=(8.6, 7.35), size=(1.55, 3.80, 1.25), yaw=0.0,
         lanes=2, lane_w=0.62, deck_z=1.05, landing_len=0.45),

    # --- 분류 테이블 (제품#1 / 제품#2 / 프레스 대기) ------------------------------
    # 슬라이드 하단과의 사이에 로봇 통로 0.75 m 를 확보하도록 y 를 잡았다.
    dict(id="SORT_TABLE", cls="box_table", name="분류 테이블",
         center=(8.6, 5.95), size=(2.45, 0.80, 0.78), yaw=0.0,
         slots=3, slot_pitch=0.80,
         slot_labels=["제품 #1", "제품 #2", "프레스 대기 제품"]),

    # --- 빈 박스 보관 랙 -------------------------------------------------------
    dict(id="EMPTY_RACK", cls="box_rack", name="빈 박스 보관",
         center=(11.6, 9.20), size=(1.30, 0.90, 1.65), yaw=0.0,
         shelves=[0.30, 0.85, 1.40]),

    # --- PRESS M/C -----------------------------------------------------------
    # 갭 프레임(C형) 프레스 80~110톤급. 개폐 정면 = -Y.
    # die_offset_y: 볼스터/다이를 베드 전면 쪽으로 당긴 로컬 y. 실제 갭프레스도
    #   다이를 전면에 두어야 투입이 되며, 이 값이 로봇의 프레스 리치를 결정한다.
    dict(id="PRESS", cls="press", name="PRESS M/C",
         center=(15.6, 9.20), size=(2.80, 2.40, 3.20), yaw=0.0,
         label="PRESS M/C", die_z=1.05, die_offset_y=-0.70),

    # --- Press 완료 제품 적재대 -------------------------------------------------
    dict(id="PRESS_OUT", cls="box_table", name="Press 완료 제품 적재",
         center=(18.40, 7.40), size=(1.70, 0.80, 0.78), yaw=0.0,
         slots=2, slot_pitch=0.80, slot_labels=["Press 완료", "Press 완료"]),

    # --- AMR Port ×3 ---------------------------------------------------------
    dict(id="AMR_PORT_P1", cls="amr_port", name="AMR Port (제품 #1)",
         center=(12.00, 3.40), size=(1.25, 0.95, 0.72), yaw=0.0,
         sign="AMR Port\n(제품 #1)"),
    dict(id="AMR_PORT_P2", cls="amr_port", name="AMR Port (제품 #2)",
         center=(14.90, 3.40), size=(1.25, 0.95, 0.72), yaw=0.0,
         sign="AMR Port\n(제품 #2)"),
    dict(id="AMR_PORT_PR", cls="amr_port", name="AMR Port (Press)",
         center=(17.80, 3.40), size=(1.25, 0.95, 0.72), yaw=0.0,
         sign="AMR Port\n(Press)"),

    # --- AMR (도킹 상태) -------------------------------------------------------
    dict(id="AMR_1", cls="amr", name="AMR #1", center=(12.00, 2.15),
         size=(0.95, 0.65, 0.32), yaw=0.0),
    dict(id="AMR_2", cls="amr", name="AMR #2", center=(14.90, 2.15),
         size=(0.95, 0.65, 0.32), yaw=0.0),
    dict(id="AMR_3", cls="amr", name="AMR #3", center=(17.80, 2.15),
         size=(0.95, 0.65, 0.32), yaw=0.0),
]

STATION_BY_ID = {s["id"]: s for s in STATIONS}

# ============================================================== 안전 펜스
# (x0,y0) -> (x1,y1) 직선 구간을 2.0 m 패널로 자동 분할. h = 패널 높이.
FENCE_SEGMENTS = [
    dict(id="FENCE_SLIDE_R", a=(10.50, 7.00), b=(10.50, 12.20), h=2.00),  # 슬라이드/랙 사이
    dict(id="FENCE_PRESS_L", a=(13.00, 6.00), b=(13.00, 12.20), h=2.00),  # 프레스 셀 좌측
    dict(id="FENCE_PRESS_B", a=(13.00, 12.20), b=(20.60, 12.20), h=2.00), # 프레스 셀 후면
    dict(id="FENCE_PRESS_R", a=(20.60, 12.20), b=(20.60, 6.00), h=2.00),  # 프레스 셀 우측
]

# ============================================================== 바닥 라인 마킹
# 개념도의 노란 통로선. (x0,y0)->(x1,y1), 폭 0.10 m, z=0.003.
LANE_WIDTH = 0.10
LANE_SEGMENTS = [
    # AMR 주행 통로 (전면 가로선 2줄)
    ((1.00, 1.30), (21.00, 1.30)),
    ((1.00, 4.90), (21.00, 4.90)),
    # 레이저 셀 작업구역 박스
    ((1.00, 4.90), (1.00, 12.60)),
    ((1.00, 12.60), (6.60, 12.60)),
    ((6.60, 12.60), (6.60, 4.90)),
    # 슬라이드/분류 셀 구역
    ((7.20, 4.90), (7.20, 12.20)),
    ((7.20, 12.20), (10.30, 12.20)),
    ((10.30, 12.20), (10.30, 4.90)),
]

# ============================================================== 로봇 / 리치
# H1 (+ 2F-85 그리퍼) reference. H1 로컬 정면 = +X → 월드 +Y 를 보려면 yaw=90.
H1_GRIPPER_USD = "C:/tmp/unitree_ros/robots/h1_description/h1_gripper.usda"
H1_CLEAN_USD   = "C:/tmp/unitree_ros/robots/h1_description/h1_clean.usd"

# JM 프로젝트 실측: 토르소 피치 없이 베이스(골반) 기준 **정면 파지 상한 0.88 m**.
# 그래서 이 월드는 "한 자리에서 다 닿는" 배치를 포기하고, 타깃마다 정면 0.75 m
# 오프셋의 **작업 스팟**을 따로 둔다(로봇이 옆으로 스텝하는 실제 작업 방식).
REACH_MAX = 0.88
STANDOFF = 0.75          # 작업 스팟 ↔ 타깃 수평 거리(정면)

_FACE = {90.0: (0.0, -1.0), -90.0: (0.0, 1.0), 0.0: (-1.0, 0.0), 180.0: (1.0, 0.0)}


def _stand(tx, ty, yaw, standoff=STANDOFF):
    """타깃 (tx,ty) 를 yaw 방향으로 바라보며 standoff 만큼 떨어져 서는 좌표."""
    dx, dy = _FACE[yaw]
    return (round(tx + dx * standoff, 3), round(ty + dy * standoff, 3))


ROBOTS = [
    # 개념도 기본 포즈: 로봇#1 은 분류 테이블(관찰자 쪽)을 향해 서 있고 슬라이드는 등 뒤.
    dict(name="Robot_1", ref=H1_GRIPPER_USD, stand=_stand(8.60, 5.95, -90.0), yaw=-90.0,
         role="슬라이드 픽 → 분류 테이블 적재 / 빈 박스 반출·적재"),
    dict(name="Robot_2", ref=H1_GRIPPER_USD, stand=_stand(15.60, 8.50, 90.0), yaw=90.0,
         role="프레스 투입·작업 / 완료 박스 적재 → AMR Port 이송"),
]

# 작업 스팟(로봇이 서는 지점 + 바라보는 방향). nav/RL 목표점으로도 쓴다.
# 전부 타깃에서 정면 0.75 m 로 자동 산출 — 임의로 손대지 말 것(리치 검증이 깨진다).
_SLIDE_PICK_Y = 7.35 + 0.45 / 2.0          # 랜딩 중앙 (SLIDE center_y + landing_len/2)
_SLIDE_LANE_DX = (0.62 + 0.06) / 2.0       # 레인 중심 오프셋 = (lane_w + 격벽)/2

STAND_SPOTS = [
    dict(id="R1_SLIDE_L", xy=_stand(8.60 - _SLIDE_LANE_DX, _SLIDE_PICK_Y, 90.0), yaw=90.0,
         desc="슬라이드 좌레인 픽"),
    dict(id="R1_SLIDE_R", xy=_stand(8.60 + _SLIDE_LANE_DX, _SLIDE_PICK_Y, 90.0), yaw=90.0,
         desc="슬라이드 우레인 픽"),
    dict(id="R1_SORT_0", xy=_stand(7.80, 5.95, -90.0), yaw=-90.0, desc="분류 테이블 제품 #1"),
    dict(id="R1_SORT_1", xy=_stand(8.60, 5.95, -90.0), yaw=-90.0, desc="분류 테이블 제품 #2"),
    dict(id="R1_SORT_2", xy=_stand(9.40, 5.95, -90.0), yaw=-90.0, desc="분류 테이블 프레스 대기"),
    dict(id="R1_RACK",   xy=_stand(11.60, 9.20, 90.0), yaw=90.0, desc="빈 박스 랙"),
    dict(id="R2_PRESS",  xy=_stand(15.60, 8.50, 90.0), yaw=90.0, desc="프레스 다이 투입/취출"),
    dict(id="R2_OUT_0",  xy=_stand(18.00, 7.40, 90.0), yaw=90.0, desc="Press 완료 적재대 1"),
    dict(id="R2_OUT_1",  xy=_stand(18.80, 7.40, 90.0), yaw=90.0, desc="Press 완료 적재대 2"),
    # AMR Port 는 **후면(+Y, 작업통로 쪽)**에서 접근한다. 전면(-Y)은 AMR 주행/도킹 공간.
    dict(id="R2_AMR_P1", xy=_stand(12.00, 3.40, -90.0), yaw=-90.0, desc="AMR Port 제품 #1"),
    dict(id="R2_AMR_P2", xy=_stand(14.90, 3.40, -90.0), yaw=-90.0, desc="AMR Port 제품 #2"),
    dict(id="R2_AMR_PR", xy=_stand(17.80, 3.40, -90.0), yaw=-90.0, desc="AMR Port Press"),
]

# 리치 검증용 (스팟 id -> 타깃 월드좌표). verify_smic_world.py 가 이걸로 거리 검사.
REACH_TARGETS = {
    "R1_SLIDE_L": (8.60 - _SLIDE_LANE_DX, _SLIDE_PICK_Y, 1.05),
    "R1_SLIDE_R": (8.60 + _SLIDE_LANE_DX, _SLIDE_PICK_Y, 1.05),
    "R1_SORT_0": (7.80, 5.95, 0.78), "R1_SORT_1": (8.60, 5.95, 0.78),
    "R1_SORT_2": (9.40, 5.95, 0.78),
    "R1_RACK": (11.60, 9.20, 0.85),
    "R2_PRESS": (15.60, 8.50, 1.05),
    "R2_OUT_0": (18.00, 7.40, 0.78), "R2_OUT_1": (18.80, 7.40, 0.78),
    "R2_AMR_P1": (12.00, 3.40, 0.72), "R2_AMR_P2": (14.90, 3.40, 0.72),
    "R2_AMR_PR": (17.80, 3.40, 0.72),
}

# ============================================================== 개념도 콜아웃
# ⚠ 아래 문구는 KETI 컨셉(안) **화면 캡처에서 판독**한 것이다. 일부 글자는
#   해상도 한계로 확정이 어려워 최선 판독값을 넣었다(원본 슬라이드로 검증 필요).
CALLOUTS = [
    # ⓘ 겐트리는 개념도에 없던 항목 — 사용자 지시로 추가(레이저↔슬라이드 사이 반송 수단).
    dict(id="CO_GANTRY", anchor="GANTRY", title="겐트리 로더",
         body="레이저 베드에서 커팅 완료 판재를 진공 리프터로 픽업\n"
              "슬라이드 고측(상단)에 적재 → 이후 중력으로 1개씩 하강\n"
              "스팬 9.5 m 문형, 브리지 하단 3.9 m (설비 상부 통과)"),
    dict(id="CO_SLIDE", anchor="SLIDE", title="커팅 제품 슬라이드 (2라인)",
         body="중력 기반 슬라이드\n커팅된 제품이 1개씩 내려와 로봇이 Pick"),
    dict(id="CO_R1", anchor="SORT_TABLE", title="로봇 #1 역할",
         body="빈 박스 반출 및 적재\n"
              "슬라이드에서 내려온 제품을 분류 테이블의 각 박스에 적재\n"
              "(양손으로 서로 다른 제품 적재)"),
    dict(id="CO_RACK", anchor="EMPTY_RACK", title="빈 박스 보관",
         body="분류 완료 후 빈 박스를 로봇 #1이 이동 및 보관"),
    dict(id="CO_R2", anchor="PRESS", title="로봇 #2 역할",
         body="프레스 대기 제품을 Press M/C에 투입\n"
              "Press 작업 수행\n"
              "완제품 박스 적재\n"
              "제품 #1, #2 완료 박스 → AMR Port로 이송\n"
              "Press 완료 박스 → AMR Port(Press)로 이송"),
    dict(id="CO_PRESS_OUT", anchor="PRESS_OUT", title="Press 완료 제품 적재", body=""),
]

# ============================================================== 태스크 흐름
# 월드가 표현해야 하는 공정 시퀀스. 이후 티처/RL 시나리오의 뼈대.
TASK_FLOW = [
    ("T1", "LASER_CUT", "GANTRY",     "레이저 커팅 완료 판재를 겐트리 진공 리프터가 베드에서 픽업"),
    ("T2", "GANTRY",    "SLIDE",      "겐트리가 슬라이드 고측에 적재 → 중력으로 1장씩 하강"),
    ("T3", "SLIDE",     "SORT_TABLE", "로봇#1: 슬라이드 픽 → 제품#1/#2/프레스대기 박스 분류 적재"),
    ("T4", "EMPTY_RACK", "SORT_TABLE", "로봇#1: 빈 박스 공급 / 가득 찬 박스 회수"),
    ("T5", "SORT_TABLE", "PRESS",     "로봇#2: 프레스 대기 제품 → Press M/C 투입"),
    ("T6", "PRESS",     "PRESS_OUT",  "로봇#2: Press 완료품 취출 → 완료 적재대 박스"),
    ("T7", "SORT_TABLE", "AMR_PORT_P1", "로봇#2: 제품#1 완료 박스 → AMR Port(제품#1)"),
    ("T8", "SORT_TABLE", "AMR_PORT_P2", "로봇#2: 제품#2 완료 박스 → AMR Port(제품#2)"),
    ("T9", "PRESS_OUT", "AMR_PORT_PR", "로봇#2: Press 완료 박스 → AMR Port(Press)"),
]


# ============================================================== 유틸
def station_bbox(s):
    """월드 축정렬 bbox (x0,x1,y0,y1). yaw!=0 이면 보수적으로 외접 사각형."""
    import math
    cx, cy = s["center"]; w, d = s["size"][0], s["size"][1]
    a = math.radians(s.get("yaw", 0.0))
    ew = abs(w * math.cos(a)) + abs(d * math.sin(a))
    ed = abs(w * math.sin(a)) + abs(d * math.cos(a))
    if s["cls"] == "gravity_slide":      # 로컬 원점이 전면 끝 → +Y 로만 뻗는다
        return cx - ew / 2, cx + ew / 2, cy - 0.25, cy + d
    return cx - ew / 2, cx + ew / 2, cy - ed / 2, cy + ed / 2


def station_ground_boxes(s):
    """바닥에 실제로 닿는 부분의 월드 축정렬 bbox 목록 [(x0,x1,y0,y1), ...].
    ground_boxes 가 없으면 전체 bbox 1개. 침범/클리어런스 검사는 **이 함수**를 쓴다
    (겐트리 브리지 같은 머리 위 구조물이 바닥 검사를 오염시키면 안 되므로)."""
    import math
    gb = s.get("ground_boxes")
    if not gb:
        return [station_bbox(s)]
    cx, cy = s["center"]
    a = math.radians(s.get("yaw", 0.0))
    ca, sa = math.cos(a), math.sin(a)
    out = []
    for (dx, dy, w, d) in gb:
        X = cx + dx * ca - dy * sa
        Y = cy + dx * sa + dy * ca
        ew = abs(w * ca) + abs(d * sa)
        ed = abs(w * sa) + abs(d * ca)
        out.append((X - ew / 2, X + ew / 2, Y - ed / 2, Y + ed / 2))
    return out


def world_boxes():
    """클리어런스 계산용 [{'center':(x,y,z),'size':(w,d,h)}, ...]."""
    out = []
    for s in STATIONS:
        for (x0, x1, y0, y1) in station_ground_boxes(s):
            out.append({"center": ((x0 + x1) / 2, (y0 + y1) / 2, FLOOR_Z + s["size"][2] / 2),
                        "size": (x1 - x0, y1 - y0, s["size"][2]), "id": s["id"]})
    return out


def clearance(px, py):
    """(px,py) 에서 가장 가까운 설비 접지부까지의 수평 거리(설비 안이면 0)."""
    best = 1e9
    for s in STATIONS:
        for (x0, x1, y0, y1) in station_ground_boxes(s):
            dx = max(x0 - px, 0.0, px - x1)
            dy = max(y0 - py, 0.0, py - y1)
            best = min(best, (dx * dx + dy * dy) ** 0.5)
    return best


def gantry_points():
    """겐트리의 픽업(레이저 베드)/드롭(슬라이드 고측) 월드 좌표."""
    g = STATION_BY_ID["GANTRY"]
    sl = STATION_BY_ID["SLIDE"]
    lz = STATION_BY_ID["LASER_CUT"]
    gy = g["center"][1]
    land = sl["landing_len"]
    run = sl["size"][1] - land
    # 슬라이드 데크 top at y = gy (경사면 선형보간)
    drop_z = sl["deck_z"] + (gy - (sl["center"][1] + land)) / run * sl["size"][2]
    return {
        "pick": (lz["center"][0], gy, 0.55),      # 레이저 슬랫 상면 부근
        "drop": (sl["center"][0], gy, round(drop_z, 3)),
        "bridge_z": g["size"][2],
    }


def slide_lane_offsets():
    """슬라이드 레인 중심의 로컬 X 오프셋. smic_models 의 레인 배치와 동일 식."""
    s = STATION_BY_ID["SLIDE"]
    n = int(s["lanes"]); pitch = s["lane_w"] + 0.06
    return [(-(n - 1) / 2.0 + i) * pitch for i in range(n)]


def slide_pick_points():
    """슬라이드 하단 랜딩의 레인별 픽 좌표(월드). [(x,y,z), ...]"""
    s = STATION_BY_ID["SLIDE"]
    cx, cy = s["center"]
    y = cy + s["landing_len"] / 2.0
    z = s["deck_z"] + PLATE["t"] / 2.0
    return [(cx + dx, y, z) for dx in slide_lane_offsets()]


def table_slot_points(sid):
    """box_table 스테이션의 박스 중심(월드) 목록."""
    s = STATION_BY_ID[sid]
    cx, cy = s["center"]
    n = s["slots"]; p = s["slot_pitch"]
    x0 = cx - (n - 1) * p / 2.0
    return [(x0 + i * p, cy, FLOOR_Z + s["size"][2]) for i in range(n)]
