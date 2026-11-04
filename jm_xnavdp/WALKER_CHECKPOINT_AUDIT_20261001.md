# 보행 체크포인트 확인 — 2026-10-01

## 후속 요청에 따른 업로드 완료

사용자 `올려줘` 요청으로 WB5 **실행용 actor**를 GitHub main에 게시했다.

- 커밋: `4143d8f27177fe16dabe02266033e789d26ac1d4`
- 경로: `models/wb5/policy_lower_corrected.pt`
- 파일 URL: https://github.com/kangbyounggwan/ISSAC_SIM_test/blob/main/models/wb5/policy_lower_corrected.pt
- 원격 파일 크기 188522 bytes, Git blob SHA `7d25af777aa28cdcab03864139a8a150c7b6e897`가 커밋 내 blob과 일치.
- 기존 WB5 SHA-256 유지. 커밋에서 추출한 깨끗한 모델 폴더로도 해시 및 CPU TorchScript 로딩/출력 [2,19] 유한값 검사 통과.
- `models/wb5/README.md`, `manifest.json`, `verify.py`, 최상위 README와 모델 1개에 한정된 ignore 예외를 함께 게시. 7개 파일만 커밋.
- 별도 배포용 main 작업폴더에서 수행하여 기존 실험 브랜치의 변경사항은 포함하지 않음.
- 전체 학습 재개용 `model_1497.pt`는 **미포함/미확보**. 보행 성능 합격이나 새 학습 수행을 뜻하지 않음.

아래 '업로드 여부 추가 확인'은 사용자 업로드 요청 **이전 시점의 조사 기록**이다.

## GitHub 업로드 여부 추가 확인

**WB5 추론 모델과 학습 재개용 원본 모두 현재 원격 Git 브랜치에 올라가 있지 않다.**

- 원격: `https://github.com/kangbyounggwan/ISSAC_SIM_test.git`
- GitHub에 직접 `ls-remote` 조회한 main: `aed2a2b31ed78b9e597a5c1786ade35b74262df8`. 로컬 origin/main과 일치하여 이 커밋의 파일 트리를 대조함.
- 원격의 나머지 2개 브랜치(`codex/jm-keti-worlds-20260928`, `kangbyounggwan/check-keti-jm-projects`) 역시 조회한 SHA와 로컬 원격 참조가 일치. 세 브랜치 모두 모델 확장자와 WB5 모델명/1497 경로 없음. 원격 tag 없음.
- 현재 로컬 브랜치에서도 WB5 파일은 `git ls-files`에 없고, `git check-ignore`는 `.gitignore`의 `artifacts/` 규칙으로 제외됐다고 확인.
- 원격 main `.gitignore`는 `artifacts/`, `checkpoints/`, `**/runs/`, `*.pt`, `*.pth`, `*.ckpt`를 제외.
- 원격 `WORLD_SNAPSHOT_MANIFEST.json`은 배포 범위를 `USD scenes and their complete local dependency graph; no training runs`로 명시. 즉 이전 업로드는 월드/자산 배포이며 보행 모델 배포가 아니었음.
- GitHub API로 확인한 Release 목록도 비어 있음. 모델이 Release에 별도 첨부됐다는 근거 없음.

현재 clone만 하면 WB5 보행 가중치를 받지 못한다. 로컬 추론 모델은 확보되어 있으나 전체 PPO 원본은 서버 경로 기록만 있으므로 둘을 별도로 확보·배포해야 한다. 이번 요청은 확인이므로 add/commit/push나 ignore 규칙 변경은 수행하지 않음.

## 결론

현재 JM 실행기에 연결된 보행 모델은 WB5 `policy_lower_corrected.pt`다. 파일 SHA-256은 고정값과 일치하고 CPU TorchScript 로딩 및 입력/출력 검사는 통과했다. 파일 무결성·실행 가능과 실제 주행 성능 합격은 별개다.

이번 작업은 로컬 읽기 검사와 이 문서 작성만 수행했다. 모델 교체/학습/시뮬레이션 재실행/서버 접속은 하지 않았다.

## 모델별 구분

| 파일 | 역할/구조 | 확인 결과 |
|---|---|---|
| `artifacts/server_training_review_20260722/wb5/policy_lower_corrected.pt` | 실제 보행 actor, 69입력 → 128 → 128 → 128 → 19출력 | TorchScript, 188522 bytes, 정상 로드, 출력 [1,19], 유한값 |
| `jm_coop4_rl/runs/h1_nav_001/model_200.pt` | 상위 랙 내비게이션 PPO, 18입력 → 3속도 명령 | actor/critic/optimizer 포함, 저장 iter 200 |
| `jm_coop4_rl/runs/h1_nav_001/model_999.pt` | 같은 상위 내비게이션의 더 뒤 저장본 | actor/critic/optimizer 포함, 저장 iter 999. 더 나은 성능은 별도 평가 필요 |
| `artifacts/xnavdp_20260930/turn_compare_official_native_neg_002/official_policy.pt` | NVIDIA 비교용 보행 actor, 69입력 → 19출력 | TorchScript, 188966 bytes, 정상 로드, 유한값. 현재 기본 모델 아님 |

WB5 19출력에는 다리뿐 아니라 몸통/팔 관절도 포함된다. 이름에 lower가 있어도 '다리만 움직이는 모델'이라고 해석하지 않는다.
상위 내비게이션의 3출력과 보행의 19출력은 서로 대체할 수 없다. 상위 999회차가 있다고 WB5 보행을 999회 학습했다는 뜻이 아니다.

## 현재 연결

- `jm_xnavdp/run_physical.py`: 기본 WB5 SHA 검사 후 `torch.jit.load`; 공식 모델은 명시적 회전 진단에서만 선택.
- 같은 실행기의 `--terminal rack_rl` 분기: 상위 **model_200.pt**를 명시적으로 로드. model_999를 자동 선택하지 않음.
- 직전 물리 검증은 `--terminal goal_pose`였으므로 model_200/999 상위 RL 도킹을 사용한 시험이 아님.
- `jm_coop4_rl/h1_rack_env.py`: WB5를 고정하고 상위 navigation 학습.
- `jm_wb8_pick_carry/config/wb8_components.json`: 같은 WB5 경로/해시를 재사용.

## 무결성

```text
WB5 actor:
e1caf0201a43a82ef8d9bb8d597bffa631c14ab6a3312938afd6eeb41e2b3e0a

상위 navigation model_200:
56fb8ddaec05da9342d0acaf0160048cd8361efc7b5b6525677c096dd06f49a9

상위 navigation model_999:
b0a1303d9c97674881aae2e6ebdbe7ab40da7f9971a6c7c20b340674ac61d60d

공식 비교 actor:
a734d8ae13b8c3690138baf8d01d7d497f987616da2c53e00a43e0ceb31452d7
```

## 이어서 학습할 원본

`JM_WB5_POSTURE_REWARD_RESULT_20260722.md`의 기록상 계보:

WB4 `model_1298.pt` → 자세 보상 변경, 4096환경에서 추가 200 iteration → WB5 `model_1497.pt` → 현재 TorchScript actor.

문서상 서버 경로:
`/workspace/jm/jm_wb3_modular/runs/wb5_posture_reward_v1/model_1497.pt`

기록된 원본 SHA-256:
`1d66c8607b39f6ed5fb3a07d72a837137c28bac51d684d5236979020746dc3c1`

현재 워크스페이스 모델 파일 검색에서 이 원본은 발견하지 못했다. 서버의 현재 존재 여부와 파일 내용은 이번에 확인하지 않았다. 로컬에는 해당 WB5의 **추론용 actor**만 확인됐다.
따라서 기존 PPO 상태를 이어 쓰려면 원본을 확보하고 actor/critic/optimizer, 관측·행동 설정, 자산 해시를 검증해야 한다. 원본 없이 actor만 복원해 critic을 새로 만드는 것은 별도 warm-start 실험이지 동일 학습의 완전 재개가 아니다.

## 성능 해석

이번 zero-input 출력 검사는 로딩/차원/유한값 검사이며 실제 보행 성공 시험이 아니다.
직전 `jm_runtime_contract_neg_001` 실측에서는 body19 PD/effort/clock 검사가 통과하고 이전 궤적도 정확히 재현됐지만, -0.3rad/s 명령의 회전량은 -21.25°이고 후반 회전 정체로 실패했다.
따라서 확인된 범위에서 '체크포인트 파일 손상' 근거는 없고, 현재 JM에서 정지·양방향 회전 성능은 아직 미검증/실패 상태다. 정책 학습 부족인지 환경 계약 차이인지는 분리 검증이 필요하다.
