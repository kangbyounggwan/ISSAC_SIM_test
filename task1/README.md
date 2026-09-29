# Task 1: RGB-D, pose estimation and grasp candidates

Linux 서버에서 JM 공장을 표시하고 RGB-D를 추출한 뒤 FoundationPose와
GraspGen-X를 각각 실행하는 도구다. Isaac Sim 화면과 결과를 SSH 터널을 통해
브라우저에서 볼 수 있다.

현재 범위는 **정적 관측과 파지 후보 생성**이다. H1 보행, 팔 IK, 접촉 파지,
리프트, 균형 제어, MES 알람 실행기는 구현하지 않았다.
GraspGen-X의 임시 그리퍼는 `franka_panda`이며 H1의 최종 장착 모델이 아니다.

## 장면과 결과의 범위

2026-09-28 첫 실험은 기존 `jm_factory_world_atlas_h1.flat.usd`의
`/World/Dadong/F2/PCBStacks/wet_recv/pcb7`를 사용했다. 이 판은
**600 × 460 × 20 mm** Cube다. 준비 도구는 저장소의 이 파일을 복사하고
텍스처 경로만 별도 레이어에서 보정해 당시 입력을 재현한다.

최신 main의 `world_products.usda`에 추가된 **510 × 340 × 1.6 mm, 0.5 kg** PCB는
다른 대상이다. 이 도구의 기본 장면을 최신 물리 PCB 실험이라고 해석하면 안 된다.
새 PCB에는 관측 위치·대상 경로·메시 및 물리 실행을 별도로 검증해야 한다.
새 H1/2F-85 USD에는 기본 H1에 없는 가상 wrist-roll 관절도 들어 있다.

FoundationPose와 GraspGen-X는 모두 시뮬레이터 정답 마스크를 사용한다.
FoundationPose는 각 프레임에서 독립적으로 `register`를 호출한다.
GraspGen-X는 diffusion-only 후보 200개에서 상위 40개를 저장한다.
후보 점수는 실측 파지 성공률이 아니며 양손 협조 파지도 검증하지 않았다.

## 준비

원래 검증 환경: Ubuntu 20.04, NVIDIA RTX A6000, Isaac Sim 4.5.0,
CUDA toolkit 12.4, PyTorch 2.6.0+cu124. 모델은 Python 3.11 환경 두 개로
분리한다. FoundationPose의 NumPy 2.x와 GraspGen-X의 NumPy 1.26.4를
한 환경에 설치하지 않는다.

필요한 도구: Git, Python 3.10 이상(venv 포함), Conda, C/C++ 컴파일러,
CUDA toolkit 12.4, 호환 NVIDIA 드라이버. 모델 및 Isaac Sim 사용에는 각각의
원저작자 이용 조건이 적용된다. 런타임·가중치·외부 소스는 Git에 포함하지 않는다.

저장소를 clone한 후:

```bash
cd task1
bash setup/bootstrap_task1.sh

# Isaac Sim 4.5.0을 별도로 설치하거나 기존 설치 경로 지정
export FACTOR_SIM_ROOT=/path/to/isaac-sim-4.5.0
export CUDA_HOME=/usr/local/cuda-12.4
# A6000은 8.6. 다른 GPU는 해당 아키텍처로 지정
export TORCH_CUDA_ARCH_LIST=8.6
# conda가 PATH에 없을 때만 지정
# export FACTOR_CONDA=/path/to/conda

bash setup/install_task1_foundationpose.sh
bash setup/install_task1_graspgenx.sh
runtime/task1_envs/bootstrap/bin/python setup/download_task1_models.py
runtime/usd-tools/bin/python setup/prepare_jm_world.py
```

기존 Isaac Sim 설치가 없다면 `setup/download_runtime.py URL ARCHIVE`로
공식 아카이브를 분할 다운로드할 수 있다. 검증에 사용한 Linux 4.5.0 URL:

<https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone%404.5.0-rc.36%2Brelease.19112.f59b3005.gl.linux-x86_64.release.zip>

다운로드 후 별도 디렉터리에 압축을 풀고 `FACTOR_SIM_ROOT`를 지정한다.
이 변수의 기본 경로는 `task1/runtime/isaac-sim-4.5.0`이다.
자동 설치 과정에서 sudo나 시스템 드라이버 변경은 수행하지 않는다.

외부 소스·가중치 revision은 bootstrap/download 스크립트에 고정했다.
FoundationPose의 Google Drive 가중치는 revision API가 없어 상위 공식 URL을
사용한다. 모델 실행 시 공식 구형 체크포인트 로딩을 위해
`TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1`을 해당 프로세스에 설정한다.

## 관측과 모델 실행

아래 출력 폴더는 아직 존재하지 않는 이름을 사용한다. 명령은 모두 `task1/`에서 실행한다.
GPU 기본값은 0이며 GUI와 별도 GPU를 쓰려면 환경변수로 바꾼다.

```bash
FACTOR_GPU=0 bash setup/run_jm_render.sh outputs/render_run01
FACTOR_CAPTURE_GPU=0 bash setup/run_task1_capture.sh outputs/capture_run01
FACTOR_MODEL_GPU=0 bash setup/run_task1_models.sh outputs/capture_run01 outputs/model_run01
```

관측은 RGB, 광축 깊이(m), K, 정답 마스크, 물체 OBJ 및 변환을 저장한다.
`T_A_B`는 B 좌표의 열벡터를 A 좌표로 옮긴다. C는 OpenCV 카메라축이다.
깊이로 복원한 점과 USD 메시의 거리를 검사해 좌표/단위 오류를 검출한다.

웹 결과 게시:

```bash
runtime/usd-tools/bin/python setup/build_task1_results.py \
  --observations outputs/capture_run01 \
  --foundationpose outputs/model_run01/foundationpose \
  --graspgenx outputs/model_run01/graspgenx \
  --output outputs/task1_results
cp outputs/model_run01/grasp_preview.usda outputs/task1_results/grasp_preview.usda
bash setup/run_jm_gui.sh
```

## Mac / Linux 브라우저에서 보기

개인 PC에서 본인의 서버 접속 정보로 실행한다:

```bash
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 -p SSH_PORT USER@SERVER
```

브라우저에서 <http://127.0.0.1:8765/>를 연다.
VS Code Remote SSH의 Ports 탭에서 8765를 전달해도 된다.
`/task1/`은 결과 페이지이며 결과 게시 전에는 404가 정상이다.
상단 버튼이 보이지 않으면 전체 화면을 종료하고 새로고침한다.

GUI는 서버의 localhost에만 바인딩한다. Kit 전체 창 캡처와 입력 전달을
사용하며 NVIDIA WebRTC 클라이언트가 아니다. 임의 파일/명령 실행 API는 없다.
Task 1 파일 제공은 지정 결과 디렉터리와 확장자로 제한한다.
원본 USD 대신 session layer에 관찰 카메라와 후보 메시를 추가한다.

FoundationPose 표시: 정답은 자홍색 실선, 추정은 청록색 점선이다.
색상 재생성은 모델을 실행하거나 저장된 자세/평가 수치를 바꾸지 않는다.

## 검증 기록

- [첫 실행 결과와 실패 사례](docs/TASK1_FIRST_EXPERIMENT.md): 2026-09-28 기존 서버의 실제 GPU 실행.
- [모듈 검토](docs/TASK1_MODEL_INTEGRATION_REVIEW.md): 당시 적용 범위와 후속 조건.
- [PR 준비 검사](docs/PR_VALIDATION.md): 코드 이동 후 수행한 검사와 수행하지 않은 범위.

이전 실험 문서의 절대 실행 경로나 GPU 번호보다 이 README의 명령을 우선한다.
이전 실험 데이터·영상·가중치는 PR에 포함하지 않는다.
