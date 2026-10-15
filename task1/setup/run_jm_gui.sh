#!/usr/bin/env bash
set -euo pipefail
FACTOR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FACTOR_OUTPUT="${1:-$FACTOR_ROOT/outputs/jm_gui_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$FACTOR_OUTPUT"
FACTOR_OUTPUT="$(cd -- "$FACTOR_OUTPUT" && pwd)"
FACTOR_SIM_ROOT="${FACTOR_SIM_ROOT:-$FACTOR_ROOT/runtime/isaac-sim-4.5.0}"
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u PYTHONPATH -u LD_LIBRARY_PATH -u PYTHONEXE \
  PYTHONNOUSERSITE=1 VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json \
  "$FACTOR_SIM_ROOT/python.sh" "$FACTOR_ROOT/setup/serve_jm_gui.py" \
  --world "$FACTOR_ROOT/workspace/jm_world/jm_factory_world.usda" \
  --output "$FACTOR_OUTPUT" --gpu "${FACTOR_GPU:-0}" --port "${FACTOR_GUI_PORT:-8765}" \
  2>&1 | tee "$FACTOR_OUTPUT/isaacsim.log"
