# Task 1 첫 모델 연결 실험

실행일: 2026-09-28. 현재 서버에서 데이터 추출, 모델 설치, 가중치 다운로드와 실제 GPU 추론을 완료했다.
이 결과는 고정 시점의 인식·파지 후보 생성 검증이다. H1의 보행, 팔 이동, 그리퍼 접촉·들기 실험은 포함하지 않는다.

## 화면 확인

기존 Mac 브라우저의 `http://127.0.0.1:8765/`를 새로고침한다.

- **PCB 파지 후보**: 실제 Isaac Sim 장면에 첫 시점의 상위 5개 후보 그리퍼를 표시한다.
  파란색은 최고 점수 후보, 주황색은 나머지 후보다. 각각 독립된 대안이며 동시에 실행할 다섯 손이 아니다.
  가상 Franka 그리퍼를 H1에 장착하거나 동작시키는 화면은 아니다.
- **Task 1 실험 결과**: `/task1/`에서 원본, 정답 마스크, FoundationPose 및 GraspGen-X 결과를 시점별로 비교한다.
  행렬과 점수를 담은 NPZ와 전체 결과 JSON도 내려받을 수 있다.
- 기존 VS Code/SSH의 8765 포트 전달을 그대로 사용한다. 추가 포트는 필요하지 않다.

## 대상과 입력

- 원본 대상: `/World/Dadong/F2/PCBStacks/wet_recv/pcb7`.
- 형상: 원본 USD의 0.60 × 0.46 × 0.02m Cube. 세부 회로·부품이 있는 실제 PCB CAD가 아니다.
- 시점: 같은 물체를 보는 세 개의 정지 카메라. 세 번째는 주변 구조물로 크게 가려진다.
- RGB 640×480, 광축 방향 깊이[m], 카메라 K, 인스턴스 정답 마스크, 카메라·물체 정답 변환, 일치하는 OBJ를 저장했다.
- 깊이에서 복원한 대상 점과 원본 표면의 거리 99백분위: 1.13 / 1.26 / 1.16mm.
- 포즈 정답은 평가에만 사용했다. FoundationPose는 각 영상에서 `register`를 독립 호출하며 정답 포즈를 입력하지 않았다.
- **두 모델 모두 시뮬레이터 정답 마스크를 사용했다.** 대상 검출·분할 모델까지 평가한 결과가 아니다.

## 실제 추론 결과

| 시점 | FoundationPose 위치 오차 | 회전 오차, 직육면체 대칭 고려 | 자세 추정 시간 | GraspGen-X 후보 생성 시간 | 저장한 파지 후보 |
|---|---:|---:|---:|---:|---:|
| 1 | 2.45mm | 0.29° | 19.07초 | 0.66초 | 40개 |
| 2 | 1.53mm | 0.29° | 1.09초 | 0.15초 | 40개 |
| 3, 가림 | 98.25mm | 88.05° | 1.06초 | 0.15초 | 40개 |

첫 FoundationPose 실행 시간에는 NVDiffRast/Warp의 초기 GPU 코드 준비가 포함된다.
표의 시간은 모델 로딩과 센서 획득을 제외한 이번 실행 값이다. 지연시간 벤치마크가 아니다.
두 번째 시점은 일반 회전 오차가 약 180°이나, 무늬 없는 직육면체의 대칭을 고려하면 약 0.29°다.
위아래 면이 다른 실제 PCB에서는 이 대칭 가정을 그대로 적용하면 안 된다.

FoundationPose의 세 번째 시점은 큰 자세 오차를 보였다. 이를 숨기지 않고 결과에 포함했다.
작업 실행기로 연결할 때는 가림, 추정 일관성과 재관측 여부를 판단하는 단계가 필요하다.

GraspGen-X는 임시 `franka_panda` 그리퍼로 **diffusion-only** 200개 후보를 생성한 뒤 점수 상위 40개를 저장했다.
공식 기본값의 GraspMoE와 구별한다. 점수 임계값은 -1로, top-k 결과를 반환하도록 설정했다.
이번 저장 후보의 점수는 모두 0.7 이상이지만, 이 값은 실제 성공 확률이나 실행 가능성의 증명이 아니다.
부분 관측에서도 높은 점수가 나왔으므로 주변 구조물·팔 도달성 검증을 생략해서는 안 된다.

## 설치 구성

- `runtime/task1_envs/foundationpose`: Python 3.11, PyTorch 2.6.0+cu124, NumPy 2.4.6.
  PyTorch3D v0.7.9, NVDiffRast v0.3.3, Eigen/Boost 기반 mycpp를 빌드했다.
- `runtime/task1_envs/graspgenx`: Python 3.11, PyTorch 2.6.0+cu124, NumPy 1.26.4.
- `third_party/FoundationPose`: `a1b694b83e633c2cb6115b9063d940a687759392`.
- `third_party/GraspGenX`: `b9429097728cb1c430dd78b92edf17ba318aad03`.
- FoundationPose의 공식 Google Drive refiner/scorer 가중치를 받았다.
- GraspGen-X 공식 안내의 Hugging Face 모델 `7c834043c11a11417e31d6d5ea9355801e40a2c1`,
  gripper_descriptions `19a03c00d19aeaf052d0f6801f0041982d676e8a`의 Franka 관련 자산을 받았다.
- Isaac Sim은 기존 별도 런타임을 사용한다. GUI GPU 0, 관측 추출 GPU 1, 추론 GPU 2에서 실행했다.
  추론과 추출은 종료했고 GUI만 남겨 둔다.

각 환경의 `uv pip check`가 통과했다. 설치 목록은 `outputs/task1_setup/*_freeze.txt`에 있다.
공장 Git 저장소와 두 모델의 추적 소스 파일은 수정하지 않았다. 연결 코드는 `setup/`에 있다.
PyTorch 2.6과 이전 형식의 공식 가중치 로더를 연결하기 위해 모델 실행 프로세스에서
`TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1`을 사용했다.
첫 FoundationPose 시도의 Ninja PATH 오류는 어댑터가 해당 환경의 bin을 PATH에 추가하도록 수정했다.
실패 로그는 남겨 두었으며 최종 성공 결과는 `task1_foundationpose_v2`다.

## 다시 실행

서버 터미널에서 실행한다. 출력 디렉터리는 매번 새로 만들어 기존 결과를 보존한다.

```bash
cd task1

# 저장된 관측으로 두 모델을 다시 실행. 기본 GPU 2.
bash setup/run_task1_models.sh

# 새 RGB-D 관측 묶음 생성. 기본 GPU 1.
bash setup/run_task1_capture.sh
```

새 관측을 모델에 넣으려면 `run_task1_models.sh`의 첫 인자로 해당 폴더를 전달한다.
`FACTOR_MODEL_GPU`, `FACTOR_CAPTURE_GPU`로 사용할 GPU를 지정할 수 있다.
새 실행은 게시된 첫 결과 페이지를 자동으로 덮어쓰지 않는다.
설치 명령은 `setup/install_task1_foundationpose.sh`, `setup/install_task1_graspgenx.sh`에,
가중치 다운로드는 `setup/download_task1_models.py`에 기록했다.

## 산출물과 검증

- 관측: `outputs/task1_observations_v1/`.
- FoundationPose: `outputs/task1_foundationpose_v2/`.
- GraspGen-X: `outputs/task1_graspgenx_v1/`.
- 게시용 결과: `outputs/task1_results/`.
- 실행·설치 로그: `outputs/task1_setup/`.
- 원본/가중치/입력 SHA-256, 커밋, 좌표 검사: `outputs/task1_setup/runtime_manifest.json`.
- 브라우저 시점 선택·이미지 검사: `outputs/task1_setup/browser_result_checks.json`.

좌표는 `T_A_B`가 B의 열벡터를 A로 옮기는 규칙이다. C는 OpenCV 카메라 좌표이며 단위는 미터다.
`T_W_C @ T_C_O_gt = T_W_O`, `T_W_G = T_W_C @ T_C_G`와 출력 회전행렬을 확인했다.
원본 PCB 스케일은 OBJ 정점에 반영했으며 포즈에는 중복 적용하지 않는다.

## 다음 단계

1. 정답 마스크를 대상 검출·분할 결과로 교체해 인식 전체를 평가한다.
2. 세부 PCB 형상·물성·허용 접촉 부위와 최종 로봇·그리퍼를 확정한다.
3. 파지 후보에 팔 IK, 팔·몸통·랙 충돌, 접근 경로를 검사하고 실제 접촉·들기·유지를 검증한다.
4. 접근 위치 선정은 고정 위치 기준선부터 구성한다. 현재 완료된 것은 고정 관측이며 베이스 이동은 실행하지 않았다.
5. 보행 체크포인트를 받으면 관측·행동 정의와 로봇 모델을 대조해 이동 구간을 연결한다.
   UniLM-Nav 공식 실행 코드는 확보되지 않아 이번 실행에 포함하지 않았다.
