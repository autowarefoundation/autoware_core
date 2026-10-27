#!/bin/bash
set -e

DEVCONTAINER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${DEVCONTAINER_DIR}/build" "${DEVCONTAINER_DIR}/install" "${DEVCONTAINER_DIR}/log"
