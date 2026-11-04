# H1 제자리 회전: 모델·학습 구조 조사 — 2026-09-30

후속 **실제 비교는 완료**했다: [공식 H1 / WB5 실험 결과](H1_TURN_COMPARISON_20260930.md). 아래는 비교 전 조사 기록이다. 현재는 공식 가중치 단독 교체를 권고하지 않으며, 환경·초기화·제어 조건을 먼저 통일해야 한다. 새 RL 학습은 미실행이다.

## 추천 결론

**현재 가중치를 더 학습하기 전에 NVIDIA 공식 H1 원본 예제를 비교 기준으로 실행하는 것이 우선**이다. '회전 모델'은 보통 내비게이션 모델이 아니라 `(전진 속도, 횡속도, yaw 회전속도)`를 받아 발과 관절을 움직이는 저수준 보행 정책이다. X-NavDP/A*는 유지하고 이 계층을 비교할 수 있다.

현재 코드는 WB5 `policy_lower_corrected.pt`, 69차원 관측, 19개 관절 출력을 사용한다. 이번에는 모델 검색과 소스 대조까지만 했으며 외부 가중치 다운로드·교체나 새 RL 학습을 수행하지 않았다. 아래 외부 모델의 JM 성공률은 아직 측정하지 않았다.

## 후보 비교

| 후보 | 확인한 제공 내용 | 현재 프로젝트와 관계 | 우선순위 |
|---|---|---|---|
| NVIDIA H1 Flat Terrain Policy | 공식 사전학습 정책, env 설정, 좌우 회전 데모 | 로컬 5.1 예제는 69 입력/19 출력으로 현재 인터페이스와 가장 가까움. 로봇 형상·관절 순서·물성 일치는 별도 확인 | 1: 원본 예제 비교 |
| Unitree RL Lab — H1 Velocity | 제조사 공식 Isaac Lab 학습 코드, yaw 속도 명령·보행 위상·발 보상 | 회전 전용 학습/재학습 구조 참고에 적합. 관측과 action scale이 달라 기존 WB5에 코드만 섞으면 안 됨 | 2: 학습 구조 후보 |
| Unitree RL Gym — H1 motion.pt | 제조사 H1 사전학습 파일과 MuJoCo 배포 설정 | 41 입력/10 출력, LSTM 계열 학습 구성. 현재 69/19 인터페이스에 그대로 교체 불가 | 3: 별도 비교 후보 |
| HumanoidVerse | H1 10/19DoF 보행 학습 및 다중 시뮬레이터 비교 코드 | sim-to-sim 차이 분석에 유용. 이번 조사에서 즉시 사용할 특정 H1 회전 체크포인트는 확인하지 못함 | 연구 참고 |

### 1. NVIDIA 공식 H1: 가장 가까운 비교 기준

5.1 문서는 H1 평지 보행 정책의 좌우 회전 조작과 학습/배포 설정을 설명한다. [공식 문서](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/isaac_lab_tutorials/tutorial_policy_deployment.html), [정책 배포 예제](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/ext_isaacsim_robot_policy_example.html).

로컬 설치 소스를 직접 확인했다:

- `C:/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/h1.py`: `H1FlatTerrainPolicy`, 자산 루트의 `/Isaac/Samples/Policies/H1_Policies/h1_policy.pt`와 `h1_env.yaml`을 로드. 69 관측/19 이전 action, action scale 0.5.
- `C:/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/tests/test_h1.py`: `test_robot_turn_command`가 `(0,0,1)`을 3초 적용하고 yaw 변화가 1.5rad보다 큰지 검사한다.

**테스트 코드가 있다는 사실은 현재 JM에서 통과했다는 뜻이 아니다.** 또한 그 테스트는 위치 드리프트/음의 방향/저속 0.30rad/s까지 인증하지 않는다. 원본 예제와 원본 자산부터 실행한 뒤 JM에 동일 정책을 이식해 차이를 봐야 한다.

웹의 최신 문서는 IO_descriptors/RobotPolicyRunner 방식으로 바뀌어 있으므로, 현재 설치된 5.1의 H1FlatTerrainPolicy와 섞지 않는다. 설치 버전에 맞는 공식 구현을 기준으로 삼는다.

### 2. Unitree RL Lab: 회전 학습 구조 참고

공식 저장소는 H1을 지원한다. H1 설정에는 heading 목표 변환 대신 yaw 속도를 직접 주는 명령, gait phase 관측, 좌우 발 위상 보상 및 발 미끄러짐/들림/원치 않는 접촉 보상이 있다. 현재 WB5와 관측·정규화·action scale이 다르므로 별도 환경으로 비교해야 한다. 제조사 코드라는 이유만으로 랙 앞 정밀 제자리 회전을 보장하지 않는다. [공식 저장소](https://github.com/unitreerobotics/unitree_rl_lab), [H1 실제 설정](https://github.com/unitreerobotics/unitree_rl_lab/blob/main/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/h1/velocity_env_cfg.py).

### 3. Unitree RL Gym: 사전학습 파일은 있지만 다른 인터페이스

`deploy/pre_train/h1/motion.pt` 제공을 확인했다. 배포 YAML은 41 관측/10 action, action scale 0.25 및 별도 PD/정규화를 사용한다. 학습 설정은 recurrent actor를 지정한다. 현재 19관절 정책 자리에 파일만 갈아 끼울 수 없으며 관절 매핑·관측·주기/상태 관리·상체 제어를 따로 맞춰야 한다. [사전학습 파일 위치](https://github.com/unitreerobotics/unitree_rl_gym/tree/main/deploy/pre_train/h1), [배포 설정](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_mujoco/configs/h1.yaml), [학습 설정](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/legged_gym/envs/h1/h1_config.py).

### 4. 연구용 대안

[HumanoidVerse](https://github.com/LeCAR-Lab/HumanoidVerse)는 H1 보행을 여러 시뮬레이터에서 비교하는 코드가 있어 배포 차이를 분석하는 참고 자료다. 특정 JM용 회전 체크포인트를 확보한 상태는 아니다.

[OmniH2O 논문](https://arxiv.org/abs/2406.08858)과 [공식 코드](https://github.com/LeCAR-Lab/human2humanoid)는 H1 전신 동작 추종/모방 학습 계열이다. 사람처럼 회전하는 시범 동작을 확장하는 연구 방향으로는 관련 있지만, 현재의 속도 명령 기반 회전 문제만 해결하려고 전체 파이프라인을 교체하기에는 범위가 크다. 우선 도입 대상으로 추천하지 않는다.

## 현재 학습에서 점검할 부분

보관된 학습 코드와 현재 설치된 Isaac Lab 기본 H1 설정에는 명시적인 '선속도 0 + 비영 yaw' 샘플 그룹이 없다. standing 그룹은 yaw도 0이 된다. 또 `feet_air_time_positive_biped`는 선속도 명령이 0.1m/s 이하이면 보상을 꺼서 순수 회전에는 기여하지 않는다.

다만 이것만으로 실패 원인을 확정할 수 없다. 같은 계열의 기본 정책에도 회전 기능이 있으므로, 추가 학습 중 기능 저하 또는 개조 자산·바닥·제어 설정 차이도 함께 확인해야 한다. 과거 서버의 전체 라이브러리 버전은 현재 로컬 소스만으로 확정하지 않았다.

## 권장 실험 순서 — 아직 실행하지 않은 계획

1. **원본 기준선:** NVIDIA 원본 H1+원본 policy/env로 정지·좌우 회전을 확인. 단순 yaw 변화뿐 아니라 위치 드리프트와 정지 후 잔여속도를 기록한다.
2. **배포 대조:** 동일 정책을 JM H1에 연결해 원본과 비교. 관절 순서·기본값·구동기·질량/관성·마찰·발 collider를 하나씩 대조하며, 관절 각도를 임의로 바꾸지 않는다.
3. **명령 분포 실험:** 배포 설정이 맞는 경우에만, 기존 가중치를 보존한 별도 학습에서 순수 좌/우 회전 표본과 정지→회전→정지 전이를 추가한다. 이 실험에서는 보상까지 동시에 바꾸지 않는다.
4. **보상 실험:** 이후 별도 실험에서 yaw 명령에도 발 교대 디딤을 보상하도록 검토한다. yaw 추종·위치 유지·과한 미끄러짐·넘어짐을 따로 기록한다. 기존 전진 성능 퇴행도 평가한다.
5. **래킹 연결:** 통로에서 양방향 90°/180°와 걷기→정지→회전이 통과한 뒤 랙 앞 도킹에 다시 연결한다. 첫 시험의 2초 정지 명령에도 약 24cm 이동했으므로, 정지 명령을 주는 것과 안정 정지 달성은 분리해서 평가해야 한다.

각 비교는 같은 seed·물성·목표·평가 기준으로 진행한다. 안전/성공 기준을 실패 후 자동 완화하지 않는다. H1용 체크포인트도 개조된 H1에 대한 검증 없이 안전한 모델로 간주하지 않는다.
