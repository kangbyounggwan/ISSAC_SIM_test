# 목표 방향 회전·정지 학습 반영 — 2026-10-02

최신 결과: **1,000회 학습·6개 영상 생성 완료, 회전 평가 6개 모두 FAIL. 운영 교체 없음.** [결과 확인 보고서](GOAL_POSE_RESULT_20261002.md)를 먼저 참조한다. 아래 시작 시점 기록은 이력으로 보존한다.

## 목적과 구분

사용자 요청에 따라 [회전 학습 리서치](TURNING_TRAINING_RESEARCH_20261001.md)를 바탕으로 별도 `goal_pose_v1` 실험을 추가했다. **운영 WB5와 기존 `train_001` 체크포인트는 보존한다.**

이번 것은 목표 자세를 입력하는 WB5 확장 실험이다. Unitree 공식 H1 설정 전체 이식이나 `No More Marching` 논문 재현이라고 주장하지 않는다. 목표 입력, 보상, 명령/정지 스케줄을 함께 바꾼 구조 변경이므로 이전 실험과 단일 변수 비교라고 해석하지 않는다. 기존 최소 변경 실험 코드는 그대로 남긴다.

## 데이터 흐름

```text
WB5 원본 actor 69입력 / 19출력
  → 기존 입력 가중치 보존 + 추가 7입력 가중치는 0으로 초기화
  → 76입력 / 19출력 actor (초기 출력 원본과 일치 검사)

초기 위치·방향 + 좌우 목표 45/90/180°
  → 고정 anchor, 부호가 있는 목표 회전량
  → 현재 위치/방향 오차 + 정지/회전/제동 단계
  → 제한된 속도 힌트 + 목표 관측
  → actor → 동일한 관절 목표/PD → 실제 물리 시뮬레이션
  → 오차·접촉·낙상·정지 유지 기록 → PPO 업데이트
  → 별도 체크포인트 → 720p 평가 → 수치 검증 전 운영 교체 금지
```

root를 직접 회전하거나 움직여 성공시키지 않는다. 위치/관절 초기화는 episode reset에서만 발생한다. 영상에서는 자동 reset을 차단해 낙상·실패를 숨기지 않는다.

## 학습 구성

| 항목 | 고정 설정 |
|---|---|
| 장비 | 로컬 RTX 4080 SUPER / Isaac Sim 5.1 / Isaac Lab |
| 원본 | `artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt` |
| 원본 SHA256 | `e1caf0201a43a82ef8d9bb8d597bffa631c14ab6a3312938afd6eeb41e2b3e0a` |
| 관측 | WB5 69개 유지 + 상대 XY, yaw sin/cos, 부호 있는 잔여 회전량/π, pose 모드, 단계 = 76개 |
| 행동 | 기존 BODY19 관절, action scale 0.5 유지 |
| 그룹 | 정지 25%, 좌회전 25%, 우회전 25%, 기존 보행 25% |
| 회전 목표 | 각 episode에 45°/90°/180° 중 표본 추출, 좌우 부호 유지 |
| 시간 | 정착 2초 → 목표 회전 12초 → 제동/유지 4초, 총 18초 |
| 시간 경계 | 50Hz 정수 step 100/700/900, 부동소수 시간 누적 사용 안 함 |
| 속도 힌트 | 회전 단계에서 yaw 오차 비례, 최대 ±0.6rad/s; 위치 복귀 힌트 최대 축별 ±0.15m/s |
| 제동 | 마지막 4초는 속도 명령 0. 목표를 미달해도 목표각을 바꾸지 않음 |
| physics / policy | 200Hz / 50Hz |
| 본 학습 | 512환경 × 1,000 PPO 반복, 반복당 환경별 24step |
| warm start | actor만 이식, critic/optimizer/std 새로 생성. 완전 resume 아님 |
| 초기 25회 | critic 준비, actor와 action std 고정 |
| 이후 | actor/critic PPO 업데이트, 50회 간격 저장 |

추가 목표 관측은 보행 그룹에서 0이다. 이것이 학습 후 보행 능력을 보장하지는 않으므로 별도 보행 평가가 필요하다. signed yaw를 누적하여 +180°와 −180°의 방향을 구분한다. 목표 좌표나 각도는 실패 후 자동 수정하지 않는다.

## 보상 변경과 한계

- 기존 자세·높이·속도 추종·발 미끄러짐 항목 유지.
- 발 air-time 보상을 선속도뿐 아니라 순수 yaw 명령에서도 활성화. 정지 명령에서는 꺼진다.
- 회전 단계의 목표 yaw 오차 감소 항목 추가(weight 2).
- 고정 anchor에서 벗어나는 XY 거리 제곱 비용 추가(weight −4).
- 목표 자세에서 양발 접촉·저속·안정 높이/기울기를 만족하는 항목 추가(weight 2).
- 마지막 제동 단계의 잔류 선속도/yaw 속도 비용 추가(weight −2).
- pose 그룹에서 anchor 1m 이탈은 실패 종료. 이는 성공 허용오차가 아니다.

주기적 발 디딤 clock이나 강제 관절 궤적은 추가하지 않았다. 모든 항목은 실험 시작 전에 고정하며 실행 중 성능에 맞춰 가중치/목표/허용오차를 자동 완화하지 않는다.

## 성공 판정 — 초기 평지용 기준, 랙 인증 아님

완료 episode가 다음을 모두 만족해야 성공으로 센다.

- 마지막 제동/유지 단계에서 위치 오차 ≤ 10cm, 방향 오차 ≤ 5°.
- 수평 속력 ≤ 0.05m/s, 절대 yaw 속도 ≤ 0.05rad/s.
- 몸체 기울기 ≤ 10°, root 높이 ≥ 0.80m.
- 좌우 발 접촉력 각각 > 5N.
- 위 상태가 연속 최소 0.75초(38 제어 step) 유지.
- episode 중 최대 XY 이탈 ≤ 20cm, 낙상/실패 종료 없음.

이것은 이전 실험 허용치를 실패에 맞춰 수정한 것이 아니라 **새 실험에 미리 고정한 평지 판정**이다. 랙 간섭·제품 운반 가능 영역에서 검증된 기준은 아니다. 정지 성공과 좌회전 성공, 우회전 성공을 섞지 않는다. 각 방향에서도 45/90/180°별 성공/실패와 최종 오차를 별도 집계한다.

## 실행 전 검증 및 기록

| 검증 | 결과 |
|---|---|
| 순수 수학/호환성 단위 검사 | 기존 3개 + 신규 8개 = 11개 통과 |
| preflight_001 | 제한 실행에서 Isaac Sim 외부 캐시 접근 실패. 물리 실험 실패로 해석하지 않음 |
| preflight_002 | 32환경/900step, 종료 오류 0, 관절/관측 검사 통과 |
| preflight_003 | 초기화 명령 재계산과 각도별 기록을 보완한 동일 설정 재검사. 32환경/900step, 실패 종료 0 |
| 원본 회전 성능 | preflight_003 좌 0/8, 우 0/8 성공. 새 구조만 붙였다고 회전 성공으로 표시하지 않음 |
| 짧은 학습 | `smoke_001`: 30/30회 정상 종료, actor 확장·PPO·체크포인트·TorchScript 내보내기 확인 |
| 가중치 검사 | 초기 출력 최대 차이 0.0. actor 최대 변화 0.00518565, 추가 목표 입력 가중치 최대 절댓값 0.00272059 |
| 영상 초기 검사 | `video_smoke_001`은 RGB render product 초기화 전 촬영을 시도해 검은 프레임 검사에서 차단. 초기화만 수정해 `video_smoke_002` 재촬영 |

사전 검사 통과는 runtime 준비이며 회전 성능 통과가 아니다. 첫 2초 종료 오류, 비정상 수치, 관측/행동 크기, 관절 순서, PD/effort/default를 검사한다. 실패나 timeout으로 초기화되기 **전** 결과를 저장한다.

`status.json`에는 PID와 갱신 시간이 포함된다. TRAINING 문자열만으로 프로세스 생존을 판단하면 안 된다. 본 학습은 smoke 가중치를 이어 쓰지 않고 원본 WB5에서 다시 시작한다.

## 파일과 재실행

- `goal_math.py`, `test_goal_pose.py`: 좌표·각도·단계·판정·가중치 확장 검사.
- `goal_commands.py`: 목표 생성, 속도 힌트, 부호 있는 회전량, episode 통계.
- `goal_env.py`: 추가 관측·보상·종료, reset 전 결과 기록.
- `train_goal_pose.py`: 사전 검사, 학습, 체크포인트, 상태 기록.
- `record_goal_pose.py`: 좌/우 45·90·180°의 전신 720p 촬영. 영상·프레임·telemetry·결과 동시 저장.
- `verify_recordings.py`: MP4 전체 프레임 디코딩 및 해상도/시간 연속성 검사.
- `run_goal_pose_pipeline.ps1`: 1,000회 학습 정상 완료 확인 후 6개 방향/각도 영상 평가와 파일 QA를 순서대로 실행. 실패 시 다음 단계로 넘어가지 않음.

```powershell
& C:/isaacsim/python.bat -u jm_wb5_turn/train_goal_pose.py --out artifacts/wb5_goal_turn_20261002/preflight_NEW --num_envs 32 --preflight-only --headless --device cuda:0
& C:/isaacsim/python.bat -u jm_wb5_turn/train_goal_pose.py --out artifacts/wb5_goal_turn_20261002/train_NEW --num_envs 512 --max_iterations 1000 --readiness artifacts/wb5_goal_turn_20261002/preflight_NEW --headless --device cuda:0
& C:/isaacsim/python.bat -u jm_wb5_turn/record_goal_pose.py --policy artifacts/wb5_goal_turn_20261002/train_NEW/policy_goal_pose_candidate.pt --out artifacts/wb5_goal_turn_20261002/video_NEW --headless --device cuda:0
& C:/isaacsim/python.bat jm_wb5_turn/verify_recordings.py artifacts/wb5_goal_turn_20261002/video_NEW
```

학습 설정이나 의존 소스가 사전 검사 뒤 변경되면 학습을 차단한다. 기존 출력 디렉터리를 덮어쓰지 않는다. `policy_goal_pose_candidate.pt`는 76입력이라 기존 WB5의 69입력 호출부에 바로 넣으면 안 된다.

## 남은 검증

본 학습 결과, 기존 보행 성능 비교, 여러 seed/heading, 걷기→회전 연결, 랙/운반/자체 충돌은 별도 검증 대상이다. 이번 버전은 독립된 정지-회전-정지 episode와 보행 그룹을 학습하며, **한 episode 안의 보행→회전→보행/좌우 반전은 아직 구현하지 않았다.**

Fable 실행 파일은 이번 환경에서 확인되지 않아 새 공동 검토는 하지 못했다. 외부와 합의된 설계라고 표시하지 않는다.

## 짧은 학습 후보의 720p 검증 결과

`video_smoke_002`는 30회 학습 후보의 고정 초기 자세/단일 seed 검사다. 실제 actor 업데이트는 마지막 5회뿐이며 본 학습 성능으로 제시하지 않는다.

| 목표 | 최종 실제 회전 | 최종 방향 오차 | 최대 XY 이탈 | 최종 속력 | 성공 |
|---|---:|---:|---:|---:|---|
| 좌 +90° | +36.81° | +53.19° | 22.75cm | 0.0028m/s | FAIL |
| 우 −90° | −16.52° | −73.48° | 13.48cm | 0.0013m/s | FAIL |

넘어짐 종료는 없었으나 방향이 부족하고 좌회전은 최대 이탈 기준도 넘었다. 정지 속력만 낮다고 성공으로 판정하지 않는다. 좌회전은 14초 약 +44.9°에서 제동 뒤 18초 +36.8°로 일부 되돌아오는 프레임을 확인했다. 최종 후보에서 다시 점검할 항목이다.

두 영상은 각각 1280×720 / 25fps / 451프레임 / 18.04초, telemetry 901개/50Hz다. 전체 디코딩에서 누락·암흑 프레임 없음. 전신·발이 포함되는 중간/종료 프레임을 직접 확인했다. 파일 QA 통과와 동작 성공은 별도다.

## 본 학습 실행

2026-10-02 00:19 KST에 `train_001` 파이프라인을 로컬에서 시작했다. 512환경/1,000회 한도이며 smoke 후보를 재사용하지 않고 원본 actor에서 시작한다. 파이프라인 PID 49504, 학습 프로세스 초기 PID 23616. PID는 이 실행에만 해당하며 생존 확인을 함께 해야 한다.

- 학습 상태: `artifacts/wb5_goal_turn_20261002/train_001/status.json`
- 실제 설정·해시: 같은 폴더의 `manifest.json`, `environment_config.json`
- 전체 진행: `artifacts/wb5_goal_turn_20261002/train_001_pipeline.log`
- 오류 출력: `artifacts/wb5_goal_turn_20261002/train_001_pipeline.err.log`
- 종료 후 예정 영상: `artifacts/wb5_goal_turn_20261002/train_001_video/{left_45,right_45,left_90,right_90,left_180,right_180}/`

마지막 영상 경로는 학습이 정상 완료되어야 만들어지는 예정 산출물이다. 현재 완료·회전 성공·운영 적용을 뜻하지 않는다. 실행 중에는 모델/환경 코드나 판정치를 바꾸지 않는다.

00:20 KST 실측: **50/1000회, ACTOR_CRITIC_PPO**, PID 23616 실제 생존 확인. 초기 25회 critic 준비를 지나 actor 업데이트가 진행 중이며 이식 초기 출력 차이는 0.0이다. 이 기록 이후의 진행은 `status.json`과 실제 PID를 함께 확인한다.
