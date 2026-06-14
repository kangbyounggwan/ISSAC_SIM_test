#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dadong_3f_layout.py — 다동(메인공장) **3층 장비 배치도** 디지털화.

도면(110 x 56 그리드)을 분석해 PCB 공정 라인 장비를 구조화한 단일 소스.
- 그리드 1 cell = 0.5 m.  도면 110 cols x 56 rows  ==  다동 55 m x 28 m.
- 그리드 → 월드 좌표 매핑이 world_geometry.py 의 DADONG(x:14–69, y:1–29) 과 정합.
- 좌(서) 코어 = EV/계단/화장실, 우(동) 코어 = EV/계단/화장실.
- 상단(북) = 습식 라인 + PSR 정면기,  중앙 = DF 라인 + 노광/LDI,  남동 룸 = PSR현상/터널건조/Silk.

각 장비: id, kr, en, w(=X m), d(=Y m), cc(center col 1..110), cr(center row 1..56), zone.
치수는 라벨의 실제 장비 풋프린트(mm→m). 도면 박스는 가독성 위해 더 크게 그려져 있으나
여기서는 실치수를 사용(AMR 충돌체용). cc/cr 은 1차 추정 → 검증 렌더로 확인/보정.
"""

# ----------------------------------------------------------------- 그리드 ↔ 월드
GRID = dict(cols=110, rows=56, cell=0.5,
            x0=14.0, x1=69.0,   # 월드 X 범위(다동 동서)
            y0=1.0,  y1=29.0)   # 월드 Y 범위(다동 남북)
FLOOR_Z = 6.0   # 상층(2F) 슬래브 top z. 건물은 2층(사용자 확인) → 본 라인은 다동 2F 공정층.
TOP_IS_NORTH = True   # 도면 위쪽 = 북(+Y)


def grid_to_world(cc, cr):
    """그리드 중심(col,row) → 월드 (x,y) m. col 1=서, row 1=위."""
    x = GRID["x0"] + (cc - 0.5) * GRID["cell"]
    if TOP_IS_NORTH:
        y = GRID["y1"] - (cr - 0.5) * GRID["cell"]
    else:
        y = GRID["y0"] + (cr - 0.5) * GRID["cell"]
    return x, y


# ----------------------------------------------------------------- 색상(zone)
ZONE_COLOR = dict(
    wet=(0.91, 0.78, 0.31),       # 습식(현상/부식/박리/건조)
    io=(0.91, 0.72, 0.72),        # 투입/수취
    psr=(0.95, 0.84, 0.45),       # PSR 정면기/현상기
    df=(0.95, 0.84, 0.45),        # DF 정면기/라미네이션
    printer=(0.84, 0.80, 0.66),   # 인쇄기
    expose=(0.91, 0.79, 0.55),    # 노광
    silk=(0.85, 0.79, 0.66),      # Silk Auto
    ldi=(0.72, 0.72, 0.72),       # LDI
    dryer=(0.95, 0.84, 0.45),     # 최종건조/터널건조/가건조
    measure=(0.93, 0.82, 0.42),   # 3차원측정
    room=(0.80, 0.84, 0.80),      # 교반실/닦의실/LDI 룸
    util=(0.88, 0.88, 0.85),      # A/C, Air, AW보관, 테이블
    misc=(0.93, 0.90, 0.80),      # CP, S/E, 동분, 랙
    reagent=(1.00, 1.00, 1.00),   # 약액(현상액/염산 등)
)

# zone별 대표 장비 높이(z, m) — 3D 충돌체용
ZONE_H = dict(wet=2.2, io=1.6, psr=2.4, df=2.4, printer=1.8, expose=2.2, silk=2.4,
              ldi=2.6, dryer=2.5, measure=1.8, room=2.8, util=1.2, misc=1.2, reagent=1.2)

# ----------------------------------------------------------------- 장비 목록
# (id, kr, en, w[X m], d[Y m], cc, cr, zone)
EQUIPMENT = [
    # ---- 습식 라인(상단/북) : 마일러→현상→부식→박리→건조외→수취 (좌→우 흐름) ----
    ("mylar",    "마일러",   "Mylar",        1.0,   1.8,  8.0,  6, "wet"),
    ("develop",  "현상",     "Develop",      8.781, 1.8, 17.8,  6, "wet"),
    ("etch",     "부식",     "Etch",        10.835, 1.8, 37.4,  6, "wet"),
    ("strip",    "박리",     "Strip",        6.569, 1.8, 54.8,  6, "wet"),
    ("dryetc",   "건조 외",  "Dry/etc",      4.666, 1.8, 66.0,  6, "wet"),
    ("wet_recv", "수취",     "Receive",      1.0,   1.8, 71.7,  6, "io"),
    # 약액/조작(라인 위)
    ("dev_liq",  "현상액",   "Dev-soln",     1.0,   1.0, 19.0,  2, "reagent"),
    ("hcl",      "염산",     "HCl",          1.0,   1.0, 30.0,  2, "reagent"),
    ("chlorate", "염소산",   "Chlorate",     1.0,   1.0, 35.0,  2, "reagent"),
    ("strip_liq","박리액",   "Strip-soln",   1.0,   1.0, 50.0,  2, "reagent"),
    ("ctrl_pnl", "조작 판넬","Ctrl-panel",   3.0,   1.0, 84.0,  3, "reagent"),

    # ---- PSR 정면기 라인(우상단) : 투입→PSR정면기→수취 (좌→우 흐름) ----
    ("psr_in",   "투입",     "Input",        1.5,   1.8, 77.0,  6, "io"),
    ("psr_lvl",  "PSR 정면기","PSR-leveler", 10.5,  1.8, 88.0,  6, "psr"),
    ("psr_recv", "수취",     "Receive",      1.5,   1.8,100.0,  6, "io"),
    ("cp1",      "CP",       "CP",           2.0,   1.0, 79.0,  3, "misc"),
    ("cp2",      "CP",       "CP",           2.0,   1.0, 86.0,  3, "misc"),
    ("se1",      "S/E",      "S/E",          1.0,   1.0, 80.0, 11, "misc"),
    ("dongbun1", "동분",     "Cu-powder",    1.0,   1.0, 86.0, 11, "misc"),

    # ---- 인쇄기(우측, 북동) x3 ----
    ("printer1", "인쇄기",   "Printer",      2.0,   1.6,105.0,  5, "printer"),
    ("printer2", "인쇄기",   "Printer",      2.0,   1.6,105.0, 12, "printer"),
    ("printer3", "인쇄기",   "Printer",      2.0,   1.6,105.0, 18, "printer"),

    # ---- DF 라인(중앙) : 투입→DF정면기→자동라미네이션→수취 (우→좌 흐름) ----
    ("df_in",    "투입",     "Input",        1.5,   1.8,100.0, 24, "io"),
    ("df_lvl",   "DF 정면기","DF-leveler",  11.0,   1.8, 71.0, 24, "df"),
    ("lamin",    "자동 LAMINATION","Auto-Lam",5.5,  1.8, 46.0, 24, "df"),
    ("df_recv",  "수취",     "Receive",      1.5,   1.8, 37.0, 24, "io"),
    ("dongbun2", "동분",     "Cu-powder",    1.0,   1.0, 66.0, 21, "misc"),
    ("se2",      "S/E",      "S/E",          1.0,   1.0, 85.0, 21, "misc"),
    ("cp3",      "CP",       "CP",           2.0,   1.0, 96.0, 19, "misc"),

    # ---- 노광 / LDI / Dry Film(중앙 좌) ----
    ("exp1",     "노광1",    "Exposure-1",   1.5,   2.5, 22.0, 15, "expose"),
    ("exp2",     "노광2",    "Exposure-2",   1.5,   2.5, 29.0, 15, "expose"),
    ("ldi_room", "LDI 룸",   "LDI-room",     3.945, 5.01,47.5, 16, "room"),
    ("ldi",      "LDI",      "LDI",          1.74,  3.41,47.0, 16, "ldi"),
    ("rack85",   "8x5",      "Rack-8x5",     0.8,   0.5, 54.0, 13, "misc"),
    ("rack86",   "8x6",      "Rack-8x6",     0.8,   0.6, 54.0, 15, "misc"),
    ("dryfilm2", "Dry Film 2","DryFilm-2",   2.5,   1.0, 58.0, 12, "misc"),
    ("dryfilm1", "Dry Film 1","DryFilm-1",   2.5,   1.0, 65.0, 12, "misc"),

    # ---- 좌(서) 유틸리티 컬럼 ----
    ("table_w",  "테이블",   "Table",        1.0,   4.0,  3.0,  5, "util"),
    ("ac_w",     "A/C",      "A/C",          1.0,   1.0,  3.0,  9, "util"),
    ("aw_w",     "AW 보관",  "AW-store",     1.0,   2.0,  3.0, 14, "util"),
    ("aws_w",    "A/W Setting","AW-Setting", 1.5,   1.5,  3.0, 20, "util"),
    ("su_la",    "수 L/A",   "su-L/A",       1.0,   1.0, 14.0, 21, "util"),
    ("table_a",  "테이블",   "Table",        2.5,   1.0, 33.0, 22, "util"),
    ("ac1",      "A/C",      "A/C",          1.0,   1.0, 18.0, 22, "util"),

    # ---- 남동 룸(녹색) : PSR현상기/터널건조/Silk/노광/3차원측정 ----
    ("final_dry1","최종건조","Final-dry",    1.5,   1.5, 76.0, 24, "dryer"),
    ("final_dry2","최종건조","Final-dry",    1.5,   1.5, 82.0, 24, "dryer"),
    ("mixing",   "교반실 및 잉크보관","Mixing/Ink",3.0,1.5,87.0,24, "room"),
    ("aw_s",     "AW 보관",  "AW-store",     1.0,   2.0, 90.5, 24, "util"),
    ("cleanrm",  "닦의실",   "Clean-rm",     2.5,   1.5, 73.0, 24, "room"),
    ("air1",     "Air",      "Air",          1.0,   1.5, 78.0, 24, "util"),
    ("aws_s",    "A/W Setting","AW-Setting", 1.5,   1.5, 90.0, 30, "util"),
    ("dongbun3", "동분",     "Cu-powder",    1.0,   1.0, 88.0, 28, "misc"),
    ("se3",      "S/E",      "S/E",          1.0,   1.0, 88.0, 32, "misc"),
    ("psr_dev",  "PSR 현상기","PSR-developer",1.8, 11.5, 80.0, 37, "psr"),
    ("psr_dev_in","투입",    "Input",        1.8,   1.5, 80.0, 51, "io"),
    ("tunnel",   "터널 건조기","Tunnel-dryer",1.6, 11.0, 91.0, 38, "dryer"),
    ("tunnel_rv","수취",     "Receive",      1.5,   1.6, 91.0, 51, "io"),
    ("table_s",  "테이블",   "Table",        1.0,   4.0, 70.0, 34, "util"),
    ("cmm",      "3차원측정","3D-CMM",       1.5,   2.0, 71.0, 41, "measure"),
    ("silk2",    "Silk Auto-2","Silk-2",     2.0,   3.0, 76.0, 50, "silk"),
    ("silk1",    "Silk Auto-1","Silk-1",     2.0,   3.0, 82.0, 50, "silk"),
    ("exp2b",    "노광2",    "Exposure-2",   1.5,   2.5, 88.0, 50, "expose"),
    ("exp1b",    "노광1",    "Exposure-1",   1.5,   2.5, 94.0, 50, "expose"),
    ("predry",   "가건조",   "Pre-dry",      1.5,   1.5, 99.0, 52, "dryer"),
    ("ac2",      "A/C",      "A/C",          1.0,   1.0, 70.0, 52, "util"),
    ("ac3",      "A/C",      "A/C",          1.0,   1.0,102.0, 36, "util"),
    ("table_e",  "테이블",   "Table",        1.0,   4.0,102.0, 43, "util"),

    # ---- 복도(중앙) ----
    ("load_tbl", "적재 TABLE","Load-table",  3.5,   1.0, 50.0, 27, "util"),
]

# 공정 흐름(화살표): (from_id, to_id) — 시각화/AMR 라우팅 참고
FLOW = [
    ("develop", "etch"), ("etch", "strip"), ("strip", "dryetc"), ("dryetc", "wet_recv"),
    ("psr_in", "psr_lvl"), ("psr_lvl", "psr_recv"),
    ("df_in", "df_lvl"), ("df_lvl", "lamin"), ("lamin", "df_recv"),
    ("psr_dev_in", "psr_dev"), ("tunnel", "tunnel_rv"),
]


def world_boxes():
    """장비 → 월드 박스 parts. (id, kr, center(x,y,z), size(w,d,h), color, zone)."""
    out = []
    for (eid, kr, en, w, d, cc, cr, zone) in EQUIPMENT:
        x, y = grid_to_world(cc, cr)
        h = ZONE_H.get(zone, 1.5)
        z = FLOOR_Z + h / 2.0
        out.append(dict(id=eid, kr=kr, en=en, center=(x, y, z), size=(w, d, h),
                        color=ZONE_COLOR[zone], zone=zone))
    return out


if __name__ == "__main__":
    from collections import Counter
    print("equipment:", len(EQUIPMENT))
    print("by zone  :", dict(Counter(e[7] for e in EQUIPMENT)))
    for b in world_boxes()[:3]:
        print(b["id"], "->", tuple(round(v, 2) for v in b["center"]), tuple(b["size"]))
