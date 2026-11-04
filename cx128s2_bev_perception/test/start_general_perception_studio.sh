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
STUDIO_DIR="$LAB_TEST_DIR/general_perception_studio"
python3 "$STUDIO_DIR/build_reader.py" --state "$LAB_STATE_DIR"
STUDIO_DEPS="$LAB_STATE_DIR/native_deps"
mkdir -p "$STUDIO_DEPS"
if [[ ! -d "$STUDIO_DEPS/imgui/.git" ]]; then
  git clone --depth 1 --branch v1.91.9b-docking https://github.com/ocornut/imgui.git "$STUDIO_DEPS/imgui"
fi
if [[ ! -d "$STUDIO_DEPS/implot/.git" ]]; then
  git clone --depth 1 --branch v0.16 https://github.com/epezent/implot.git "$STUDIO_DEPS/implot"
fi
[[ "$(git -C "$STUDIO_DEPS/imgui" rev-parse HEAD)" == 4806a1924ff6181180bf5e4b8b79ab4394118875 ]] || { echo "Unexpected ImGui version" >&2; exit 1; }
[[ "$(git -C "$STUDIO_DEPS/implot" rev-parse HEAD)" == 18c72431f8265e2b0b5378a3a73d8a883b2175ff ]] || { echo "Unexpected ImPlot version" >&2; exit 1; }
cmake -S "$STUDIO_DIR" -B "$LAB_STATE_DIR/native_build" -DSTUDIO_DEPS="$STUDIO_DEPS" -DCMAKE_BUILD_TYPE=Release
cmake --build "$LAB_STATE_DIR/native_build" -j "${GENERAL_STUDIO_BUILD_JOBS:-4}"
exec "$LAB_STATE_DIR/native_build/general_perception_studio" --state "$LAB_STATE_DIR" "$@"
