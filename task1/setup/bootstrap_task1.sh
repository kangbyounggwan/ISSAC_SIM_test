#!/usr/bin/env bash
# Fetch pinned sources and small setup tools; models and Isaac Sim remain separate.
set -euo pipefail
FACTOR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FACTOR_BOOT="$FACTOR_ROOT/runtime/task1_envs/bootstrap"
mkdir -p "$FACTOR_ROOT/third_party" "$FACTOR_ROOT/outputs/task1_setup" "$FACTOR_ROOT/models"
if [[ ! -x "$FACTOR_BOOT/bin/python" ]]; then
  "${FACTOR_BOOTSTRAP_PYTHON:-python3}" -m venv "$FACTOR_BOOT"
fi
"$FACTOR_BOOT/bin/python" -m pip install uv==0.12.19 gdown==6.4.0 huggingface-hub==2.0.0
FACTOR_UV="$FACTOR_BOOT/bin/uv"
export UV_CACHE_DIR="$FACTOR_ROOT/.downloads/uv-cache"
export UV_PYTHON_INSTALL_DIR="$FACTOR_ROOT/runtime/python"
if [[ ! -x "$FACTOR_ROOT/runtime/usd-tools/bin/python" ]]; then
  "$FACTOR_UV" venv --python 3.13 --seed "$FACTOR_ROOT/runtime/usd-tools"
fi
"$FACTOR_UV" pip install --python "$FACTOR_ROOT/runtime/usd-tools/bin/python" \
  usd-core==26.8 Pillow==12.3.0 numpy==2.4.6

fetch_source() {
  local name="$1" url="$2" revision="$3"
  local target="$FACTOR_ROOT/third_party/$name"
  if [[ -e "$target" ]]; then
    [[ "$(git -C "$target" rev-parse HEAD)" == "$revision" ]] || {
      echo "Existing $name checkout differs from the pinned revision; leaving it untouched." >&2
      exit 1
    }
    [[ -z "$(git -C "$target" status --porcelain --untracked-files=no)" ]] || {
      echo "Tracked source changes in $name; leaving them untouched." >&2
      exit 1
    }
    return
  fi
  git init "$target"
  git -C "$target" remote add origin "$url"
  git -C "$target" fetch --depth 1 origin "$revision"
  git -C "$target" checkout --detach FETCH_HEAD
}

fetch_source FoundationPose https://github.com/NVlabs/FoundationPose.git a1b694b83e633c2cb6115b9063d940a687759392
fetch_source GraspGenX https://github.com/NVlabs/GraspGenX.git b9429097728cb1c430dd78b92edf17ba318aad03
printf 'Setup tools and pinned sources are ready. See task1/README.md for runtime, models and inference.\n'
