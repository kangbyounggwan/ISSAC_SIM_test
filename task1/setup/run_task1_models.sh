#!/usr/bin/env bash
# Repeat inference on saved observations. Each run is separate from the published first experiment.
set -euo pipefail
FACTOR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FACTOR_OBSERVATIONS="${1:-$FACTOR_ROOT/outputs/task1_observations_v1}"
FACTOR_RUN="${2:-$FACTOR_ROOT/outputs/task1_models_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname -- "$FACTOR_RUN")"
mkdir "$FACTOR_RUN"
FACTOR_RUN="$(cd -- "$FACTOR_RUN" && pwd)"
export CUDA_VISIBLE_DEVICES="${FACTOR_MODEL_GPU:-0}"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.4}"
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.6}" MAX_JOBS="${MAX_JOBS:-8}" OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
cd "$FACTOR_ROOT"
"$FACTOR_ROOT/runtime/task1_envs/foundationpose/bin/python" setup/run_task1_foundationpose.py \
  --observations "$FACTOR_OBSERVATIONS" --output "$FACTOR_RUN/foundationpose" \
  --repo "$FACTOR_ROOT/third_party/FoundationPose" > "$FACTOR_RUN/foundationpose.log" 2>&1
"$FACTOR_ROOT/runtime/task1_envs/graspgenx/bin/python" setup/run_task1_graspgenx.py \
  --observations "$FACTOR_OBSERVATIONS" --output "$FACTOR_RUN/graspgenx" \
  --models "$FACTOR_ROOT/models" > "$FACTOR_RUN/graspgenx.log" 2>&1
"$FACTOR_ROOT/runtime/usd-tools/bin/python" setup/build_task1_grasp_preview.py \
  --geometry "$FACTOR_RUN/graspgenx/preview_geometry.json" --output "$FACTOR_RUN/grasp_preview.usda"
printf 'Inference results: %s\n' "$FACTOR_RUN"
