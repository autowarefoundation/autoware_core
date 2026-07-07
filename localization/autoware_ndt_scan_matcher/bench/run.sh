#!/usr/bin/env bash
# Copyright 2024 Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# L3 offline NDT align-replay benchmark runner (see plan/ndt_bench.md).
# Builds the opt-in bench executable (both engines) with colcon, runs it to produce a JSON of
# per-align latency samples, then renders a self-contained HTML report.
#
# Run from the workspace root (/autoware_workspace), inside the dev container:
#   bash src/core/autoware_core/localization/autoware_ndt_scan_matcher/bench/run.sh [ITERS] [WARMUP] [INTERVAL]
#
# Optional env:
#   TASKSET="taskset -c 2"   pin to an isolated core for a stable tail
#   OUT_DIR=/path            where JSON + HTML land (default: the bench/ dir)

set -euo pipefail

ITERS="${1:-200}"
WARMUP="${2:-20}"
INTERVAL="${3:-0.2}"

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${OUT_DIR:-$BENCH_DIR}"
JSON="$OUT_DIR/ndt_bench.json"
HTML="$OUT_DIR/report.html"
PKG=autoware_ndt_scan_matcher

# Workspace root = where colcon builds and writes build/. Derive it by stripping the package's known
# path within the colcon `src/` tree from this script's location. This is depth-independent (works
# wherever the workspace is placed) AND immune to ancestor directories that happen to be named "src"
# (unlike a "cut at the first /src/" heuristic or a fixed "N levels up"). Override with WS_ROOT=...
# for a relocated/non-standard layout.
PKG_REL="src/core/autoware_core/localization/${PKG}/bench"
if [[ -z "${WS_ROOT:-}" ]]; then
  if [[ "$BENCH_DIR" == */"$PKG_REL" ]]; then
    WS_ROOT="${BENCH_DIR%/"$PKG_REL"}"
  else
    WS_ROOT="$PWD"
    echo "[bench] WARNING: bench dir is not at the canonical '$PKG_REL' path;" >&2
    echo "[bench]          assuming WS_ROOT=$WS_ROOT — override with WS_ROOT=... if wrong." >&2
  fi
fi

echo "[bench] workspace: $WS_ROOT"
if [[ ! -d "$WS_ROOT/src" ]]; then
  echo "[bench] ERROR: $WS_ROOT has no src/ — not a colcon workspace root (set WS_ROOT=...)." >&2
  exit 1
fi
cd "$WS_ROOT"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

echo "[bench] building $PKG (Release, NDT_USE_RUST=ON, NDT_BUILD_BENCH=ON) ..."
colcon build --packages-select "$PKG" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DNDT_USE_RUST=ON -DNDT_BUILD_BENCH=ON

EXE="$WS_ROOT/build/$PKG/ndt_bench_replay"
if [[ ! -x "$EXE" ]]; then
  # Fall back to searching the build tree (in case of a custom colcon --build-base layout).
  EXE="$(find "$WS_ROOT/build" -type f -name ndt_bench_replay -perm -u+x 2>/dev/null | head -n1 || true)"
fi
if [[ -z "$EXE" || ! -x "$EXE" ]]; then
  echo "[bench] ERROR: ndt_bench_replay not found under $WS_ROOT/build" >&2
  echo "[bench]        (did the build enable -DNDT_BUILD_BENCH=ON?)" >&2
  exit 1
fi

echo "[bench] running: $EXE $ITERS $WARMUP $JSON $INTERVAL"
${TASKSET:-} "$EXE" "$ITERS" "$WARMUP" "$JSON" "$INTERVAL"

# Capture the execution environment AT RUN TIME (this machine, right now) and merge it into the
# bench JSON under "env". run.sh does the build itself just above, so `c++`/`rustc` in PATH are the
# very compilers used. Values are passed via the environment (not string-interpolated) so quotes /
# backslashes in them can't corrupt the JSON.
CPU_MODEL="$(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2- | sed 's/^[[:space:]]*//')"
if [[ -z "$CPU_MODEL" ]]; then
  CPU_MODEL="$(lscpu 2>/dev/null | grep -m1 'Model name' | cut -d: -f2- | sed 's/^[[:space:]]*//')"
fi
CPU_CORES="$(nproc 2>/dev/null || echo '?')"
CPU_GOV="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo 'n/a')"
KERNEL="$(uname -sr 2>/dev/null || echo '?')"
CXX_VER="$(${CXX:-c++} --version 2>/dev/null | head -n1 || echo '?')"
RUSTC_VER="$(rustc --version 2>/dev/null | head -n1 || echo '?')"
STAMP="$(date -u '+%Y-%m-%dT%H:%M:%SZ' 2>/dev/null || echo '?')"

echo "[bench] capturing environment (CPU / compilers / kernel) ..."
BENCH_CPU="$CPU_MODEL" BENCH_CORES="$CPU_CORES" BENCH_GOV="$CPU_GOV" BENCH_KERNEL="$KERNEL" \
BENCH_CXX="$CXX_VER" BENCH_RUSTC="$RUSTC_VER" BENCH_STAMP="$STAMP" BENCH_TASKSET="${TASKSET:-none}" \
python3 - "$JSON" <<'PYEOF'
import json, os, sys
path = sys.argv[1]
with open(path, encoding="utf-8") as fh:
    data = json.load(fh)
data["env"] = {
    "CPU": os.environ.get("BENCH_CPU") or "unknown",
    "Logical cores": os.environ.get("BENCH_CORES", "?"),
    "CPU governor": os.environ.get("BENCH_GOV", "n/a"),
    "Kernel": os.environ.get("BENCH_KERNEL", "?"),
    "C++ compiler": os.environ.get("BENCH_CXX", "?"),
    "Rust compiler": os.environ.get("BENCH_RUSTC", "?"),
    "CPU pinning": os.environ.get("BENCH_TASKSET", "none"),
    "Captured (UTC)": os.environ.get("BENCH_STAMP", "?"),
}
with open(path, "w", encoding="utf-8") as fh:
    json.dump(data, fh, indent=2)
PYEOF

echo "[bench] rendering HTML ..."
python3 "$BENCH_DIR/gen_report.py" "$JSON" "$HTML"

echo "[bench] done:"
echo "  JSON: $JSON"
echo "  HTML: $HTML"
