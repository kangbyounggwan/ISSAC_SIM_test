#!/usr/bin/env bash
set -euo pipefail
FACTOR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FACTOR_ENV="$FACTOR_ROOT/runtime/task1_envs/graspgenx"
FACTOR_UV="$FACTOR_ROOT/runtime/task1_envs/bootstrap/bin/uv"
mkdir -p "$FACTOR_ROOT/outputs/task1_setup"
[[ -x "$FACTOR_UV" ]] || { echo "Run setup/bootstrap_task1.sh first" >&2; exit 1; }
export UV_CACHE_DIR="$FACTOR_ROOT/.downloads/uv-cache"
export UV_PYTHON_INSTALL_DIR="$FACTOR_ROOT/runtime/python"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.4}"
export MAX_JOBS="${MAX_JOBS:-8}" TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.6}"
if [[ ! -x "$FACTOR_ENV/bin/python" ]]; then
  "$FACTOR_UV" venv --python 3.11 --seed "$FACTOR_ENV"
fi
"$FACTOR_UV" pip install --python "$FACTOR_ENV/bin/python" \
  torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu124
"$FACTOR_UV" pip install --python "$FACTOR_ENV/bin/python" \
  -e "$FACTOR_ROOT/third_party/GraspGenX[serve]" 'setuptools<78' wheel
"$FACTOR_UV" pip freeze --python "$FACTOR_ENV/bin/python" > "$FACTOR_ROOT/outputs/task1_setup/graspgenx_freeze.txt"
printf 'GraspGen-X installation complete\n'
