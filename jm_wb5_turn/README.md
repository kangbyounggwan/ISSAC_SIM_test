# WB5 회전 추가 학습

기존 WB5 actor를 보존한 채, 정지와 좌우 제자리 회전 표본을 추가하는 **별도 로컬 학습 프로젝트**입니다. 운영 WB5를 자동 교체하지 않습니다.

## 환경과 데이터 흐름

```text
기존 WB5 TorchScript actor (69입력 / 19출력)
    → 동일 구조 PPO actor에 가중치 이식
    + 새 critic / optimizer / 탐색 표준편차
    → Isaac Lab 평지 H1 병렬 환경
    → 관측 → 속도 명령을 따르는 actor → 관절 위치 목표 → 물리 시뮬레이션
    → 보상·종료 관측 → PPO 업데이트
    → full PPO 체크포인트 + TensorBoard + history.jsonl
    → 별도 좌/우 회전·기존 보행 평가 → 승인 후에만 JM 연결
```

이것은 경로 계획기나 IK 학습이 아니라 **WB5의 저수준 보행/회전 제어기 추가 학습**입니다. 정책은 다리만이 아니라 몸통과 팔을 포함한 기존 BODY19 관절 출력을 유지합니다. 로봇 루트를 회전시키는 좌표 조작으로 회전을 대신하지 않습니다. 에피소드 초기화 시에만 위치·자세를 재설정합니다.

## 첫 실험 고정 조건

| 항목 | 설정 |
|---|---|
| 실행 장비 | 로컬 RTX 4080 SUPER / Isaac Sim 5.1 / Isaac Lab |
| 병렬 환경 | 본 학습 512개, 사전 검사 32개 |
| 명령 구성 | 정지 25%, 좌회전 25%, 우회전 25%, 기존 전진+회전 25% |
| 회전 시퀀스 | 정지 2초 → 회전 8초 → 정지 2초, 반복 |
| 순수 회전 명령 | vx=vy=0, yaw 속도 ±0.15~0.30 rad/s |
| 보행 표본 | vx 0~1 m/s, vy=0, yaw −1~1 rad/s |
| 물리/정책 주기 | 200Hz / 50Hz |
| 목표 반복 | 총 1,000 PPO 반복: critic 준비 25 + actor/critic 업데이트 975 |
| PPO | learning rate 1e−4 고정, 초기 action std 0.2, entropy 0.005 |
| 저장 | 초기 전체 체크포인트, 50 iteration 간격 및 종료 시 저장 |

WB5 posture_v1의 자세·높이 보상과 상속된 속도 추종 보상을 유지합니다. 순수 회전 표본을 늘리는 것이 이번 커리큘럼 변경입니다. 발 들기 보상의 선속도 조건은 이번에는 변경하지 않습니다. 실제 회전 반응이 정체되면 발 들기 조건을 다음 **별도 실험**으로 검토합니다. 실행 중 목표, 허용오차, 관절 제한 또는 보상을 자동 완화하지 않습니다.

## 재개 방식과 한계

- 원본 actor: `artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt`
- SHA256: `e1caf0201a43a82ef8d9bb8d597bffa631c14ab6a3312938afd6eeb41e2b3e0a`
- 기존 서버의 `model_1497.pt` 전체 PPO 체크포인트는 로컬에서 확인되지 않았습니다. 따라서 **원래 학습의 완전 resume가 아닌 actor-only warm start**입니다.
- 초기 actor 출력 일치를 수치 검증합니다. 새 critic을 준비하는 25회 동안 actor와 action std는 고정합니다.
- 로컬 JM 계열 H1 USD를 사용하며 관절 순서, 기본 자세, PD, effort를 시뮬레이터에서 재확인합니다. 그리퍼 mimic follower에는 중복 구동력을 주지 않습니다.
- 과거 서버 학습과 자산·초기 상태·optimizer까지 완전히 동일한 실험이 아닙니다. 따라서 과거 성능과의 차이를 명령 분포 하나만의 효과라고 단정하면 안 됩니다.
- 평지·자체 충돌 비활성 구성입니다. 랙 충돌, 제품 운반, 파지 중 회전은 아직 검증하지 않습니다.

## 파일

- `env.py`: 로봇·물리·관측·행동·보상·종료 설정
- `commands.py`, `schedule.py`: 균형 잡힌 명령 샘플링과 정지/회전 전환
- `train.py`: 사전 검사, 가중치 이식, 학습, 상태와 체크포인트 저장
- `test_schedule.py`: 명령 방향/시간 경계/독립성 검사
- `TRAINING_RECORD_20261001.md`: 실제 실행 이력
- `record_turn_audit.py`: 원본/중간 후보의 좌우 회전 및 정지 720p 비교
- `ROTATION_VIDEO_AUDIT_20261001.md`: 영상 실측, 중단 상태, 누락 조건 점검
- `TURNING_TRAINING_RESEARCH_20261001.md`: 회전 학습 논문·공식 H1 코드 비교, 목표 자세 정렬과 WB5 적용 범위
- `GOAL_POSE_IMPLEMENTATION_20261002.md`: 별도 76입력 목표 회전·정지 후보의 구현, 고정 판정, 실행 이력
- `GOAL_POSE_RESULT_20261002.md`: 1,000회 완료 후 6개 회전 평가 FAIL, 영상·로그 대조 및 적용 보류 근거

## 실행

작업 폴더는 `C:/Users/USER/ISSAC_SIM_test`입니다. 출력 폴더는 매 실행 새 이름이어야 합니다.

```powershell
& C:/isaacsim/python.bat -u jm_wb5_turn/train.py --out artifacts/wb5_turn_20261001/preflight_NEW --num_envs 32 --preflight-only --headless --device cuda:0
& C:/isaacsim/python.bat -u jm_wb5_turn/train.py --out artifacts/wb5_turn_20261001/train_NEW --num_envs 512 --max_iterations 1000 --readiness artifacts/wb5_turn_20261001/preflight_003 --headless --device cuda:0
```

사전 검사를 통과해야 학습을 시작할 수 있으며, 검사 이후 환경/명령 코드나 asset이 바뀌면 실행이 차단됩니다. 원본 모델을 덮어쓰지 않습니다.

## 결과 읽기와 운영 반영 기준

- `status.json`: 현재 반복과 학습 단계, 손실, 그룹별 속도/회전 오차. 종료 시 `TRAINING_COMPLETE`는 학습 완료일 뿐 회전 성공 판정이 아닙니다.
- `history.jsonl`: 반복별 기록. `active_yaw_error_mean_rps`는 회전 명령이 실제 있는 표본만 집계합니다. 정지 구간의 낮은 오차를 회전 성공으로 해석하지 않습니다.
- `manifest.json`, `environment_config.json`: 버전 해시와 실제 설정.
- `runtime_contract.json`, `warm_start.json`: 제어값과 actor 이식 일치 증거.
- `model_*.pt`: 새 full PPO 체크포인트(actor/critic/optimizer 포함).
- `policy_wb5_turn_candidate.pt`: 학습이 정상 종료된 경우 생성하는 추론용 후보.

학습 뒤 같은 seed·명령으로 원본과 후보를 각각 평가합니다. 좌우 회전량/명령 추종 오차, 제자리 이동량, 정지 잔류 속도, 낙상률, 기존 전진 보행 저하를 분리합니다. 이를 통과한 후에만 JM 랙 근처에서 재검증하고 운영 반영합니다. 영상은 정면/측면 또는 전신 3인칭 **1280×720**으로 기록하며, 아직 이 학습 러너에서 영상을 생성했다고 주장하지 않습니다.
