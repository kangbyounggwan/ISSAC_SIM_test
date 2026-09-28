# JM 공장 · KETI 실험환경

2026-09-28 로컬 월드 스냅샷. **월드와 참조 모델을 함께 포함**하며, 아래 진입점은
개발 PC의 `C:/tmp` 또는 사용자 폴더 없이 저장소 내부 상대경로로 구성됩니다.
Isaac Sim 설치와 기본 재질 라이브러리(`OmniPBR.mdl`)는 별도로 필요합니다.

## 바로 열기

현재 배포 브랜치: **`main`**.
2026-09-28 사용자 요청에 따라 검증된 `codex/jm-keti-worlds-20260928` 내용을
main에 fast-forward 병합했습니다. 기존 KETI 브랜치와 로컬 실험 파일은 보존했습니다.

```powershell
git clone --branch main https://github.com/kangbyounggwan/ISSAC_SIM_test.git
```

Isaac Sim의 **File → Open**에서 다음 파일을 엽니다. 폴더 구조를 유지해야 합니다.
월드 파일 하나만 다운로드하면 참조 모델이 누락됩니다.

| 환경 | 진입 파일 | 포함 내용 |
|---|---|---|
| JM 현재 공장 | [jm_factory_world_atlas_h1.usd](jm_factory_world_atlas_h1.usd) | 건물, 설비, 제품, 기존 Atlas/H1 배치 |
| JM H1 4대 | [jm_world_h1_fleet.usda](jm_coop4_rl/assets/jm_world_h1_fleet.usda) | H1 4대로 구성한 보행용 배치 |
| JM 링 작업대 | [jm_factory_world_h1_ring.usda](jm_factory_world_h1_ring.usda) | 도넛형 제품과 작업대 |
| JM 물리 그리퍼 | [jm_factory_world_h1_ring_physical_gripper.usda](jm_factory_world_h1_ring_physical_gripper.usda) | 물리 그리퍼 모델을 합성한 링 실험 월드 |
| KETI / SMIC | [smic_world.usd](smic_world/smic_world.usd) | 레이저 → 슬라이드 → 분류 → 프레스 → AMR 컨셉 셀 |

## 어디까지 포함되어 있나

- 최신 JM 월드 레이어와 KETI 셀, 참조하는 H1·Atlas·2F-85·링 모델.
- 로컬 의존 파일 26개, 복사 전후 SHA-256 및 의존 관계 목록.
- KETI 기존 설계 문서와 생성·검증 도구. 기존 JM 문서와 생성 도구도 보존.
- **미포함:** 학습 가중치, 학습 실행 폴더, 녹화 영상, 데이터셋, 비밀키, 로컬 가상환경.
- 따라서 **월드를 열 수 있는 스냅샷**이지, 곧바로 보행·파지 학습이 재생되는 전체 실행 백업은 아닙니다.
- 기존 생성 도구에는 개발 PC 전용 경로가 남아 있을 수 있습니다. 월드를 열기 위해 재생성할 필요는 없습니다.

## 검증과 한계

[검증 결과](WORLD_SNAPSHOT_VALIDATION.json) · [파일/해시 목록](WORLD_SNAPSHOT_MANIFEST.json)

```powershell
# 저장소 루트에서 실행. Isaac Sim 설치 위치가 다르면 경로와 --isaac-root를 변경합니다.
C:\isaacsim\python.bat tools\verify_world_snapshot.py --isaac-root C:\isaacsim
C:\isaacsim\python.bat smic_world\verify_smic_world.py
```

검증 범위는 USD 의존 파일, 합성, 좌표/단위, 원본 대비 속성·월드 배치입니다.
이번 업로드 과정에서 동적 보행·파지 실험을 새로 실행하지 않았습니다.

알려진 원본의 한계도 그대로 보존했습니다.

- **KETI H1 2대에는 로봇 collider가 없습니다.** 설비·제품 collider는 있지만,
  이 로봇으로 바로 물리 보행·파지 학습을 시작할 준비가 됐다는 뜻은 아닙니다.
- JM H1 모델 일부에는 이전 경로를 가리키는 재질 바인딩 경고가 남아 있습니다.
  원본과 복사본에서 동일하게 발생하며, 이번 작업에서 로봇 물리·모양을 수정하지 않았습니다.
- 링 배치는 기존 수평 배치를 보존했습니다. 수직 링 변경이나 리프트 성공을 의미하지 않습니다.

## 상세 자료

- [JM 건물·설비 설명](README_isaac_world.md)
- [JM 레이어 구조 — 과거 문서](WORLD_LAYERS_README.md)
- [KETI 사용 안내](smic_world/README.md)
- [KETI 설계 근거와 가정](smic_world/SMIC_WORLD_DESIGN.md)
- [외부 모델 출처와 고지](assets/vendor/README.md)
- [이번 업로드 작업 기록](WORLD_UPLOAD_20260928.md)
