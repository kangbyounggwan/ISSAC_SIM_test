# SMIC World

> 2026-09-28: [JM·KETI 통합 스냅샷](../README.md)에 필요한 H1/그리퍼 모델을 포함했습니다.
> `smic_world.usd`를 직접 열면 되며, 저장소 안의 상대경로로 참조합니다.
> 아래 과거 실행 예시의 사용자별 경로는 실제 clone 위치로 바꿔야 합니다.
> 기존 로봇 collider 부재는 보존되어 있으므로 물리 학습 준비 완료 상태로 해석하지 않습니다.

KETI 컨셉(안) 휴머노이드 셀(레이저 커팅 → 중력 슬라이드 → 분류 → 프레스 → AMR)의
Isaac Sim 월드. JM 시화 월드와 **같은 레이어드 USD 방법론**을 쓰되, 월드/스크립트는
완전히 분리되어 있다(JM 파일은 건드리지 않는다).

설계 근거·치수 가정·리치 검증은 **[SMIC_WORLD_DESIGN.md](SMIC_WORLD_DESIGN.md)** 참조.

## 빠른 시작

```bash
C:\isaacsim\python.bat verify_smic_world.py
```

```bash
C:\isaacsim\isaac-sim.bat --exec C:/Users/USER/ISSAC_SIM_test/smic_world/open_smic.py
```

전체 재생성(치수를 고친 뒤):

```bash
cmd /c "set SMIC_FORCE=1 && C:\isaacsim\python.bat build_smic_world.py"
```

개념도 대조 렌더(`render_smic/<Cam>/rgb_0000.png`):

```bash
cmd /c "set SMIC_CAMS=ConceptCam,TopCam && C:\isaacsim\python.bat render_smic.py"
```

## Blender 연동

Blender **4.5.1 LTS** 에서 실측 검증했다. 아래 수치·함정은 전부 실제로 돌려본 결과다.

### 가장 쉬운 길 — 셋업 스크립트

```bash
"C:\Program Files\Blender Foundation\Blender 4.5\blender.exe" --python C:/Users/USER/ISSAC_SIM_test/smic_world/blender_setup_smic.py
```

GUI 라면 `Scripting` 탭 > Open > `blender_setup_smic.py` > **Run Script**.
USD 임포트 + 아래 함정 3개 자동 보정 + 해상도 1920×1080 + ConceptCam 활성화까지 한 번에 한다.
옵션: `-- --obj` / `-- --both` / `-- --no-robots` / `-- --cam TopCam` / `-- --keep-lights`

### 수동으로 할 때 — 반드시 고쳐야 하는 3가지

`File > Import > Universal Scene Description` 로 `smic_world.usd` 를 열면
오브젝트 1013개, **카메라 4대와 조명 5개까지** 들어온다. 다만 그대로 렌더하면 안 된다.

| 증상 | 원인 | 수정 |
|---|---|---|
| 전부 **회색** | 메시 556개 중 **529개가 머티리얼 없음**. 이 월드의 색은 UsdShade 가 아니라 `displayColor` 프리미티브 속성이라 블렌더가 **컬러 어트리뷰트**로만 읽는다 | 머티리얼 하나 만들어 `Attribute` 노드(Type=Geometry, Name=`displayColor`) → Base Color 연결 후 전 오브젝트에 할당 |
| 렌더가 **순백** | 조명이 Isaac RTX 강도 그대로(Sun 1600 / RectLight 2600) | Sun `Strength=3.0`, Area 4개 `Power=300 W`, World 배경 회청색 `Strength=0.6` |
| 화면이 **뭉갠 회색** | 블렌더가 카메라 focalLength/aperture 를 **100배**로 들여옴(16 mm → 1600 mm). FOV 비율은 맞지만 1600 mm 렌즈라 피사계심도가 0 | lens·sensor 를 100 으로 나누고 **DOF 끄기**, Sensor Fit = Horizontal |

렌더 해상도는 **1920×1080** 으로 맞춘다 — 카메라 aperture 가 그 비율 기준이라
다른 비율이면 화각이 달라진다.

### OBJ 내보내기

이 월드는 대부분 암시적 프리미티브라 OBJ 로 나가려면 테셀레이션이 필요하다:

```bash
C:\isaacsim\python.bat export_smic_obj.py
```

| 출력 | 내용 |
|---|---|
| `export/smic_world.obj` (91 MB, 225만 verts) | 로봇 포함 — H1 메시가 용량 대부분 |
| `export/smic_world_norobots.obj` (0.3 MB, 5.6천 verts) | `SMIC_OBJ_NO_ROBOTS=1` 로 뽑은 경량본 |

기본 출력은 **Y-up**(OBJ 관행)이라 Blender **기본 설정 그대로** 임포트하면 바로 선다
(bbox 가 USD 월드 좌표와 정확히 일치하는 것까지 확인했다).
USD 좌표와 숫자를 맞추려면 `SMIC_OBJ_ZUP=1` 로 뽑고 임포트 설정을 Forward=Y / Up=Z 로.

프림마다 `o` **와** `g` 를 둘 다 적는다 — 블렌더 OBJ 임포터 기본값이
`use_split_objects=True / use_split_groups=False` 라 `g` 만 있으면 전체가
**한 덩어리**로 들어온다. `o` 덕분에 기본 설정에서 프림별 **522개 오브젝트**로 쪼개진다
(`World_Cell_Equip_AMR_1_body` 식으로 이름도 살아 있다).

색은 displayColor → MTL `Kd` 로 나가므로 **OBJ 쪽은 색이 확실하다**(USD 와 반대).
대신 **카메라·조명·물리·메타데이터는 OBJ 로 안 나간다.**
→ 실전 조합: `--both`. USD 로 카메라·조명, OBJ 로 형상·색. 좌표계가 같아 정확히 겹친다.

### 되가져오기 (블렌더에서 손본 형상을 월드에 편입)

```bash
C:\isaacsim\python.bat import_obj_asset.py my_press.obj --name PRESS_HQ --replace PRESS
```

OBJ → `assets/PRESS_HQ.usd` 로 변환(축 역변환 포함) 후 `/World/Cell/Custom/PRESS_HQ` 에
**reference** 로 건다. `--replace` 를 주면 원래 절차적 설비를 **숨김 처리**하는데,
USD 에서 visibility 는 물리와 무관하므로 **콜라이더는 그대로 살아 있다** —
블렌더 메시는 보기용, 기존 박스는 충돌 프록시. 임포트 메시에 직접 콜리전을 붙이려면
`--collision convexHull` (또는 `convexDecomposition`).

- `--at X Y Z --yaw <deg> --scale <s> --zup` 로 배치/축 조정
- `--remove <NAME>` 편입 취소(숨긴 원본 자동 복구)
- 모든 편입은 `smic_custom_assets.json` 에 기록된다. `SMIC_FORCE=1` 재빌드는 서브레이어를
  비우므로 편입이 지워진다 → 재빌드 후 `--replay` 로 한 번에 재적용.

라운드트립은 검증했다: 내보낸 OBJ 를 그대로 되가져오면 bbox 가 원본과 정확히 일치한다
(`[-1.5, -1.5, -0.2] ~ [23.5, 16.5, 6.0]`).

## 파일

| 파일 | 역할 |
|---|---|
| `smic_layout.py` | **치수·좌표 단일 진실원.** 실측이 들어오면 여기만 고친다 |
| `smic_models.py` | 설비 클래스별 절차적 모델(레이저/겐트리/슬라이드/프레스/AMR/펜스/컨테이너/판재) |
| `build_smic_world.py` | GENESIS — 6 서브레이어 생성 + 전체 author (순수 pxr, ~2 s) |
| `verify_smic_world.py` | 합성·축/단위·설비 침범·리치·제품 위치·로봇·물리 검증 |
| `render_smic.py` | 오프스크린 렌더(Kit 필요) |
| `export_smic_obj.py` | 합성 스테이지 → OBJ+MTL (Cube/Cylinder/Sphere 테셀레이션, 축 변환) |
| `import_obj_asset.py` | 블렌더 OBJ → USD 에셋 → 월드 편입 / `--replace` / `--remove` / `--replay` |
| `blender_setup_smic.py` | **블렌더 안에서** 실행 — 임포트 + 조명/카메라/머티리얼 보정 |
| `open_smic.py` | GUI 진입 |
| `world_layers.py` | 레이어 I/O 헬퍼 (`set_edit_layer` / `save` / `stamp_root_axis`) |
| `smic_world.usd` | thin 루트(진입점) + `world_*.usd(a)` 6 서브레이어 |

## 규칙 (JM 에서 넘어온 함정)

1. **`stage.Export(루트)` 금지.** 전체가 단일 파일로 뭉개진다.
   반드시 `wl.set_edit_layer(stage, key)` → 작성 → `wl.save(stage)`.
2. **upAxis/metersPerUnit 은 루트에 각인**해야 한다(`wl.stamp_root_axis`).
   서브레이어만 저장하면 Isaac 기본 Y-up/cm 가 새어 들어와 GUI 에서 바닥이 선다.
3. **치수 상수를 모델/빌드 스크립트에 두지 말 것.** 전부 `smic_layout.py`.
4. **작업 스팟 좌표를 손으로 고치지 말 것.** `_stand()` 가 타깃에서 산출한다 —
   손대면 0.88 m 리치 검증이 조용히 깨진다.
5. **지지면 콜라이더**를 빼먹으면 물건이 설비를 관통해 떨어진다.

## 현재 상태

- 설비 13(겐트리 포함) · **상부 개방** 컨테이너 11 · 펜스 13패널 · 판재 16 · 로봇 2 · 카메라 4, 프림 921
- 검증 **PASS** (리치 12/12, 설비 접지 침범 0, 펜스 간섭 0, 겐트리 브리지 여유 1.10 m,
  제품 이탈 0, 소유 강체 26 전부 콜라이더 보유)
- 경고 2건: 참조 H1 에셋(`h1_gripper.usda`)에 링크 콜라이더가 없음 →
  물리 파지·보행 전 보강 필요(로봇 에셋 문제, 월드 문제 아님)
