#!/usr/bin/env bash
# Allow the commonly used `sh start_general_debug_lab.sh` invocation too.
[ -n "${BASH_VERSION:-}" ] || exec bash "$0" "$@"
set -eo pipefail
LAB_TEST_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/sjjin/construct_ws/install/setup.bash
# The operating install still has the old numeric Crane message. Use the
# existing current-schema isolated build for this offline application.
export PERCEPTION_LAB_BUILD_DIR=${PERCEPTION_LAB_BUILD_DIR:-/home/sjjin/construct_ws/log/crane_work_extension_20261024/build/cx128s2_bev_perception}
export PERCEPTION_LAB_LIBRARY_DIR=${PERCEPTION_LAB_LIBRARY_DIR:-/home/sjjin/construct_ws/log/crane_housing_20261025/install/cx128s2_bev_perception/lib}
if [[ -n "${PERCEPTION_LAB_LIBRARY_DIR:-}" ]]; then
  export LD_LIBRARY_PATH="$PERCEPTION_LAB_LIBRARY_DIR:${LD_LIBRARY_PATH:-}"
fi
LAB_STATE_DIR=${GENERAL_DEBUG_LAB_STATE:-/home/sjjin/construct_ws/general_debug_lab}
python3 "$LAB_TEST_DIR/general_debug_lab/build_debug.py" --output "$LAB_STATE_DIR/tool"
exec python3 "$LAB_TEST_DIR/general_debug_lab/desktop.py" --state "$LAB_STATE_DIR" "$@"
