#!/usr/bin/env bash
set -euo pipefail
FACTOR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FACTOR_ENV="$FACTOR_ROOT/runtime/task1_envs/foundationpose"
export UV_CACHE_DIR="$FACTOR_ROOT/.downloads/uv-cache"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.4}"
export PATH="$CUDA_HOME/bin:$PATH"
export MAX_JOBS="${MAX_JOBS:-8}" TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.6}"
FACTOR_UV="$FACTOR_ROOT/runtime/task1_envs/bootstrap/bin/uv"
mkdir -p "$FACTOR_ROOT/outputs/task1_setup"
[[ -x "$FACTOR_UV" ]] || { echo "Run setup/bootstrap_task1.sh first" >&2; exit 1; }
if [[ ! -x "$FACTOR_ENV/bin/python" ]]; then
  "${FACTOR_CONDA:-conda}" create -y --prefix "$FACTOR_ENV" \
    --override-channels -c conda-forge python=3.11 pip 'cmake<4' ninja eigen boost-cpp pybind11
fi
"$FACTOR_UV" pip install --python "$FACTOR_ENV/bin/python" \
  torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu124
"$FACTOR_UV" pip install --python "$FACTOR_ENV/bin/python" \
  -r "$FACTOR_ROOT/third_party/FoundationPose/requirements.txt" 'setuptools<78' wheel ninja
"$FACTOR_UV" pip install --python "$FACTOR_ENV/bin/python" --no-build-isolation \
  'git+https://github.com/facebookresearch/pytorch3d.git@v0.7.9' \
  'git+https://github.com/NVlabs/nvdiffrast.git@v0.3.3'
"$FACTOR_ENV/bin/cmake" -S "$FACTOR_ROOT/third_party/FoundationPose/mycpp" \
  -B "$FACTOR_ROOT/third_party/FoundationPose/mycpp/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$FACTOR_ENV" -DPython3_ROOT_DIR="$FACTOR_ENV" \
  -DPYTHON_EXECUTABLE="$FACTOR_ENV/bin/python" -DPYBIND11_PYTHON_EXECUTABLE="$FACTOR_ENV/bin/python"
"$FACTOR_ENV/bin/cmake" --build "$FACTOR_ROOT/third_party/FoundationPose/mycpp/build" -j"$MAX_JOBS"
"$FACTOR_UV" pip freeze --python "$FACTOR_ENV/bin/python" > "$FACTOR_ROOT/outputs/task1_setup/foundationpose_freeze.txt"
printf 'FoundationPose installation complete\n'
