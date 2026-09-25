#!/bin/bash
# Source ROS and Autoware setup in interactive shells.
# The entrypoint sources them only for its own process tree, so shells
# spawned by `docker exec` (e.g. VS Code terminals) do not inherit them.
set -e

BASHRC="/home/aw/.bashrc"
MARKER="# autoware-core-devcontainer: source setup"

if grep -qF "${MARKER}" "${BASHRC}" 2>/dev/null; then
    exit 0
fi

cat >>"${BASHRC}" <<'BASHRC_EOF'

# autoware-core-devcontainer: source setup
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /opt/autoware/setup.bash ]; then
    source /opt/autoware/setup.bash
fi
BASHRC_EOF
