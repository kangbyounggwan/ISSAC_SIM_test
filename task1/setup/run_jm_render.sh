#!/usr/bin/env bash
set -euo pipefail
FACTOR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FACTOR_SIM_ROOT="${FACTOR_SIM_ROOT:-$FACTOR_ROOT/runtime/isaac-sim-4.5.0}"
FACTOR_GPU="${FACTOR_GPU:-0}"
FACTOR_OUTPUT="${1:-$FACTOR_ROOT/outputs/jm_render_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$FACTOR_OUTPUT"
FACTOR_OUTPUT="$(cd -- "$FACTOR_OUTPUT" && pwd)"
if [[ ! -x "$FACTOR_SIM_ROOT/python.sh" ]]; then
    echo "Isaac Sim runtime missing: $FACTOR_SIM_ROOT" >&2
    exit 1
fi
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u PYTHONPATH -u LD_LIBRARY_PATH -u PYTHONEXE \
    PYTHONNOUSERSITE=1 VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json \
    "$FACTOR_SIM_ROOT/python.sh" "$FACTOR_ROOT/setup/render_jm_world.py" \
    --world "$FACTOR_ROOT/workspace/jm_world/jm_factory_world.usda" \
    --output "$FACTOR_OUTPUT" --gpu "$FACTOR_GPU" 2>&1 | tee "$FACTOR_OUTPUT/isaacsim.log"
"$FACTOR_ROOT/runtime/usd-tools/bin/python" - "$FACTOR_OUTPUT/render_report.json" <<'PY'
import json, sys
from pathlib import Path
report = json.loads(Path(sys.argv[1]).read_text())
assert report["status"] == "success", report
assert len(report["views"]) == 3, report
for view in report["views"]:
    for name in view["files"]:
        assert Path(name).stat().st_size > 1000, name
print("Verified render report:", sys.argv[1])
PY
