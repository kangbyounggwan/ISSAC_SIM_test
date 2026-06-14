# 제이엠일렉트로닉스 시화공장 — Isaac Sim 월드 (층별 구조 반영)

건물도면(`건물도면_제이엠일렉트로닉스.pdf`, 32매)을 분석해 NVIDIA **Isaac Sim / Omniverse**용
USD 월드를 생성합니다. **로봇 주행/AMR 시뮬레이션**용 · 단위 m · Z-up · **층별 구획/구조 + 층 선택** 지원.

- 위치: 경기도 안산시 시화국가산업단지 5바 902 · 일반공업지역
- 대지 **4,950 m²** (75 × 66 m) · 철골조 · 지상 3층 규모 · 건폐율 53.86%

---

## 1. 파일 구성

| 파일 | 역할 |
|---|---|
| **`world_geometry.py`** | **단일 지오메트리 소스.** 층(floor)·분류(arch/struct/site/**equip**) 태그, 회전(트러스), 층별 구획·골조 모두 정의 |
| **`dadong_3f_layout.py`** | **다동 상층 공정 장비 배치도 디지털화(67점).** 도면 110×56 그리드(0.5m/cell)→월드 좌표. `world_boxes()` 가 world_geometry 의 다동 `DA-2F` 장비 레이어로 통합 |
| `render_dadong_3f.py` → `dadong_3f_layout.png` | **장비 배치 검증용 평면 렌더**(원 도면과 나란히 비교) |
| `jm_factory_world.py` | **USD 생성기.** world_geometry → `.usd/.usda` (물리·충돌체·조명·층/분류 메타 attr) |
| `export_viewer.py` → `viewer.html` | **standalone three.js 뷰어 (층 선택 + 건축/구조 토글)** |
| `export_obj.py` → `.obj/.mtl` | OBJ 내보내기 (Windows 3D 뷰어/Blender/온라인) |
| `render_iso.py` → `render_iso.png` | 전체 3D 음영 렌더 |
| `render_floors.py` → `render_floors.png` | **층별 패널 몽타주** (각 층 따로 보기) |
| `preview.py` | 2D 평면/단면 (`preview_plan.png`, `preview_section.png`) |
| `_validate_build.py` | USD 미설치 환경용 생성기 로직 검증 테스트 |

## 2. 실행

```bash
# Isaac Sim 내장 파이썬
"…\isaac-sim\python.bat" jm_factory_world.py --out jm_factory_world.usd
# 뷰어/OBJ 재생성 (USD 불필요)
python export_viewer.py     # viewer.html
python export_obj.py        # jm_factory_world.obj
python render_floors.py     # 층별 몽타주
```

## 3. 좌표계 / 단위
- meter, **Z-up**, 원점 = 대지 남서(SW) 모서리. `+X=동, +Y=북, +Z=위`. 1층 바닥 z=0.

## 4. 도면 기반 치수

| 요소 | W×D (m) | X | Y | 층/높이 |
|---|---|---|---|---|
| 대지 | 75 × 66 | 0–75 | 0–66 | — |
| **다동**(메인공장) | 55 × 28 | 14–69 | 1–29 | 2층 · 1F 6.0 / 2F 4.0 / 처마 10.0 / 용마루 12.4 · 박공 |
| └ 그리드 | X 5.0×11 / Y 7.0×4 | — | — | 기둥 0.3각, 층별 60개 |
| └ 코어 좌/우 | 5 × 14 | 14–19 / 64–69 | 8–22 | 계단+승강기+화장실 |
| **나동**(공장) | 42 × 17 | 22–64 | 45–62 | 1층 · 처마 6.0 / 용마루 7.5 |
| **가동**(부속동) | 10 × 17 | 4–14 | 45–62 | 3층 · 층고 3.3 · 평지붕 9.9 |
| 도로 | 서 20m / 동 10m | — | — | — |

## 5. 층별 구조 (★ 이번 반영)

각 부품에 **`floor`(층)** 과 **`cat`(arch 건축 / struct 구조 / site 대지 / equip 장비)** 태그 → 층 선택·구조/장비 토글 가능.

> **다동 상층(DA-2F) 공정 장비 67점**(현상/부식/박리 습식라인, DF·PSR 정면기, 자동 LAMINATION,
> LDI·노광, PSR 현상기, 터널 건조기, Silk Auto, 인쇄기 등)이 도면(`dadong_3f_layout.py`) 기반으로
> `cat="equip"` 충돌체로 배치됨. 뷰어 **장비** 버튼 / Isaac `jm:category=="equip"` 로 토글.
> 위치(`cc/cr`)는 1차 추정 → `dadong_3f_layout.png` 로 검증·보정.
USD 에선 경로 스코프(`/World/Dadong/F1`, `/F2`, `/Roof`, `/World/Gadong/F1·F2·F3` …)와
`jm:floor` / `jm:category` 어트리뷰트로 표현 → Isaac Sim 에서 층별 visibility 토글.

| 층 코드 | 내용 | 구조(철골) |
|---|---|---|
| `DA-1F` | 다동 1층 공장(개방홀) + 양끝 코어 + 행거도어 | 기둥 60 |
| `DA-2F` | 다동 2층 공장 + 2F 슬래브 + 창호 | 기둥 60 + **주거더(X) 5 + 소보(Y) 12** |
| `DA-roof` | 박공 지붕 클래딩 + 캐노피 | **트러스 래프터 24 + 용마루/처마/퍼린** |
| `NA-1F` | 나동 1층 공장(개방) + 사무 코너 | 기둥 그리드 |
| `NA-roof` | 나동 박공 지붕 | 트러스 |
| `GA-1F` | 가동 1층 **주방/식당**(주방 분리벽) + 계단실 | 천장 보 |
| `GA-2F` | 가동 2층 **사무실**(중복도 + 사무실 칸막이) | 천장 보 |
| `GA-3F` | 가동 3층 **기숙사**(중복도 + 양측 침실) | 천장 보 |
| `GA-roof` | 가동 옥상 평지붕 + 파라펫 | — |

> 다동 외벽·기둥·코어는 **층별(1F 0–6m / 2F 6–10m)로 분할**되어 한 층만 골라 볼 수 있습니다.

## 6. 뷰어에서 층 선택
`viewer.html`(브라우저) 좌상단 패널:
- **건축 / 구조 / 전체** 토글
- **전체 + 각 층 버튼**(다동 1F/2F/지붕, 나동, 가동 1~3F …) → 클릭 시 해당 층만 표시(지면은 항상 유지)

Isaac Sim 에선 Stage 트리에서 `/World/Dadong/F2` 등 스코프의 눈 아이콘으로 토글하거나,
`jm:floor == "GA-3F"` 같은 어트리뷰트로 필터링하세요.

## 7. AMR/물리
- 지면·벽·기둥·코어·칸막이·슬래브에 정적 충돌체(`UsdPhysics.CollisionAPI`). 머리 위 보/트러스는 충돌 제외(경량화).
- 중력 -Z 9.81. 로봇/센서는 사용자가 USD 열고 배치(또는 본 USD를 레퍼런스).

## 8. 가정 / 단순화
1. **나동·가동은 전용 평면도가 도면집에 없음** → 배치도 footprint + 설계개요서 용도/면적 + 표준 층고 기반.
   특히 **가동 1F식당/2F사무실/3F기숙사 실내 칸막이는 용도에 맞춘 대표 레이아웃(추정)** 입니다.
2. 나동/가동 층고·지붕형상은 표준값 가정. 실값 확인 시 `world_geometry.py`의 `NADONG`/`GADONG` 수정.
3. 다동 2F 슬래브는 전체 + 계단 보이드 단순화. 철골 부재 단면/개수는 대표 표현(상세 접합 제외).

## 9. 검증
- `_validate_build.py`: 생성기 무오류, **prim 436개**, 중복 경로 0, 다동 기둥 120(1F+2F), 가동 슬래브 4, Z 0–12.6m.
- `render_floors.png`: 층별 패널이 각 층의 구획/골조를 정확히 표현.
- (개발 PC는 conda/USD DLL 충돌로 pxr 직접 실행만 불가 → Isaac Sim 내장 파이썬 권장.)

## 10. 커스터마이즈
`world_geometry.py` 상단 상수(`SITE/DADONG/NADONG/GADONG/C`)와 층별 구획 함수만 고치면
USD·OBJ·뷰어·렌더가 **모두 동일하게** 반영됩니다. 수정 후 `python render_floors.py`로 먼저 확인하세요.
