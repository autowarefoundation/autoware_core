#!/usr/bin/bash
# Test-coverage runner for the autoware_ndt_scan_matcher_rs crate (source-based, cargo-llvm-cov).
#
# Requires (one-time): rustup component add llvm-tools-preview && cargo install cargo-llvm-cov
#
# Usage:
#   ./coverage.sh             # print a per-file summary table
#   ./coverage.sh --html      # also write an HTML report under target/llvm-cov/html
#   ./coverage.sh --lcov      # also write lcov.info (for CI upload)
#
# The `ros` feature is enabled so the Pose path + FFI shims are measured; bindgen needs the
# rosidl C headers, so ROS_INCLUDE_DIRS is derived from geometry_msgs.

cd "$(dirname "$(readlink -f "$0")")"

# geometry_msgs include dir for bindgen (build.rs). Prefer the env if already set.
if [ -z "${ROS_INCLUDE_DIRS:-}" ]; then
    if command -v ros2 >/dev/null 2>&1; then
        ROS_INCLUDE_DIRS="$(ros2 pkg prefix geometry_msgs)/include/geometry_msgs"
    else
        ROS_INCLUDE_DIRS="/opt/ros/humble/include/geometry_msgs"
    fi
    export ROS_INCLUDE_DIRS
fi

# Generated bindgen output and dependency sources are excluded from the coverage denominator.
IGNORE='ros_msgs\.rs|/build/|/registry/'

extra=()
case "${1:-}" in
--html) extra=(--html) ;;
--lcov) extra=(--lcov --output-path lcov.info) ;;
"") extra=(--summary-only) ;;
*) extra=("$@") ;;
esac

exec cargo llvm-cov --features ros --ignore-filename-regex "${IGNORE}" "${extra[@]}"
