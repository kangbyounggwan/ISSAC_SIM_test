# JM 공장 월드 — 레이어 구조 (부분 리로드용)

> 2026-09-28 업로드본의 진입점과 경로는 [통합 안내](README.md)를 우선합니다.
> 아래는 과거 레이어 분리 당시 기록입니다. 업로드본은 제품 레이어와 로봇 모델을 포함하며,
> 월드의 참조 파일은 저장소 내부 상대경로입니다. `C:/tmp`를 별도로 준비하지 않습니다.

기존 `jm_factory_world_atlas_h1.usd`는 24MB 단일 flatten 파일이라 무엇을 바꾸든 전체를
다시 로드해야 했다. 이를 **서브레이어 + reference** 합성 구조로 분해해서, 데이터셋(설비/
로봇/조명 등)을 바꿔도 **해당 레이어만 Reload** 하면 되도록 했다.

## 파일 구성

| 파일 | 내용 | 비고 |
|---|---|---|
| `jm_factory_world_atlas_h1.usd` | **thin 루트** — subLayers + defaultPrim만 (759 B) | 진입점(변경 없음) |
| `jm_factory_world_atlas_h1.flat.usd` | 원본 flatten 백업(24MB) | split의 소스 of truth |
| `world_base.usd` | 구조물(Ground/Fence/Dadong/Nadong/Gadong/Physics) | 거의 불변 |
| `world_equipment.usd` | F2/{Equip,Conveyors,PCBStacks,Levelers,Developers,Models} | **튜닝 대상** |
| `world_labels.usda` | F2/Labels(투입/배출 표지판·화살표) | |
| `world_robots.usda` | 로봇 4체 = clean USD **reference** + transform | 메시 24MB는 외부 |
| `world_staging.usda` | Lighting + 카메라(RenderCam/FloorCam/…) | 렌더 조정 |

서브레이어 강도 순서(강→약): staging → labels → equipment → robots → base.
로봇 메시(약 24MB)는 `C:/tmp/.../atlas_clean.usd`, `h1_clean.usd`에만 있고 월드는 참조만 한다.

## GUI에서 부분 갱신

1. `Window > Layers` 패널을 연다. 서브레이어 5개가 보인다.
2. **외부 편집 후 리로드**: 스크립트/텍스트로 한 레이어(예: `world_equipment.usd`)만 바꾸고,
   Layers 패널에서 그 레이어 우클릭 → **Reload Layer**. 24MB 재로드 없이 즉시 반영.
3. **GUI 직접 편집**: 편집할 레이어를 더블클릭해 **Authoring Layer(굵게)**로 지정 → 옮기고/
   바꾸고 → `Ctrl+S` 하면 그 레이어 파일만 저장된다.

## 스크립트 (레이어 인식)

모든 빌드/편집 스크립트는 이제 `world_layers.py` 헬퍼로 **해당 서브레이어에만** 기록한다
(`stage.Export`로 전체를 다시 flatten하지 않는다). 재실행해도 레이어 구조가 유지된다.

| 스크립트 | 기록 레이어 |
|---|---|
| `build_world.py` | robots(reference) + staging(조명·카메라) |
| `model_equipment.py` | equipment (Models) |
| `add_conveyors.py` | equipment (Conveyors/PCBStacks) + staging(카메라) |
| `add_levelers.py` / `add_developer.py` | equipment + staging(카메라) |
| `detail_and_label.py` | equipment(Levelers/Developers) + labels(Labels) |
| `delete_columns.py` | base (기둥 삭제) |

- 실행: `C:\isaacsim\python.bat <script>.py`
- **렌더 생략(빠른 저장만)**: `WORLD_NORENDER=1` 환경변수 → SimulationApp만 띄우고 저장 후 종료.
- 헬퍼 API (`world_layers.py`): `set_edit_layer(stage, "<base|equipment|labels|robots|staging>")`,
  `save(stage)`(서브레이어만 저장, 루트는 건드리지 않음), `ROOT`, `layer_path(key)`.

## 유지보수 도구

- `split_world.py` — flatten 백업(`*.flat.usd`)에서 레이어 세트를 **멱등**하게 재생성.
  레이어가 꼬이거나 누가 다시 flatten해버리면 이걸 돌려 깨끗이 복구한다.
- `verify_split.py` — 레이어드 합성 결과가 flatten 원본과 **prim 1:1 동일**한지 검증
  (구조 prim 1297개 일치 + 로봇 bbox 1mm 이내 + 합성 에러 0).
- `_usd_boot.py` — Kit 부팅 없이 `pxr`만 빠르게 로드(순수 USD 조작용). split/verify가 사용.

## 주의

- 새 스크립트를 만들 때 절대 `stage.Export(루트)` 하지 말 것 → 전체가 다시 24MB로 뭉개진다.
  반드시 `world_layers.set_edit_layer(...)` + `world_layers.save(stage)` 사용.
- 라벨 텍스처는 절대경로(`C:/.../textures/label_*.png`)라 레이어 위치와 무관하게 해석된다.
