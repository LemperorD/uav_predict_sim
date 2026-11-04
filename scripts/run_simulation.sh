#!/usr/bin/env bash
set -eo pipefail
SIM_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$SIM_WORKSPACE/install/setup.bash"
export PYTHONNOUSERSITE=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-$SIM_WORKSPACE/log/prediction}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$SIM_WORKSPACE/.cache/matplotlib}"
exec /usr/bin/python3 "$SIM_WORKSPACE/scripts/run_simulation.py" "$@"
