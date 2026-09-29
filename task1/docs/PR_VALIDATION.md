# Task 1 도구의 PR 준비 검사

2026-09-29. 기존 Linux 서버에서 검증한 코드를 `task1/`로 옮기고
서버 전용 경로를 제거한 뒤 수행한 검사다.

## 이번 변경에서 확인한 것

- Python 스크립트 11개 AST 구문 검사, shell 스크립트 7개 `bash -n` 통과.
- 새 checkout에서 `prepare_jm_world.py` 실행 성공.
- 원본 flat USD와 준비된 사본의 SHA-256 동일:
  `5a05d188f74edbb2535b125ffa1fb4390306cfca8e7477ab15449ce6c17a3bb1`.
- USD 합성 오류 0, Z-up, meter 단위, 로봇 네 대 메시 확인.
- 저장된 자세로 오버레이 3개를 재생성했고 기존 자홍색/청록색 PNG와 해시 일치.
  입력 자세 행렬과 결과 JSON의 해시도 보존.
- 저장된 파지 후보로 USD 메시 5개 생성. 메시의 월드 변환이 저장된 후보와 일치.
  표시용 메시에는 물리 rigid body/collision을 추가하지 않음.
- 결과 페이지 생성 성공. 세 시점의 RGB, mask, GT/pose/grasp 이미지와 NPZ 존재 확인.
- 개인 서버의 사용자 경로·IP·SSH 포트·토큰을 추가 소스에 포함하지 않음.

## 검증 범위

2026-09-28 원본 코드로 실제 GPU 렌더링, RGB-D 추출,
FoundationPose/GraspGen-X 추론과 브라우저 표시를 수행했다.
수치는 `TASK1_FIRST_EXPERIMENT.md`의 해당 실행 기록을 따른다.

PR 준비 과정에서는 기존 설치를 사용한 장면 준비 및 저장 결과 재생성을 검사했다.
새 기계에서의 전체 bootstrap/다운로드/빌드, GPU 재추론과 새 GUI 프로세스 기동은
반복하지 않았다. 신규 bootstrap 스크립트는 구문 검사 범위이다.
실행 중이던 GUI는 유지했다.

새 1.6 mm PCB, H1 보행, 팔 IK, 접촉 파지, 리프트와 보행 정책 호환성은
이번 PR의 검증에 포함되지 않는다.
