#!/usr/bin/env bash
[ -n "${BASH_VERSION:-}" ] || exec bash "$0" "$@"
set -eo pipefail
LAB_TEST_DIR="$(cd "$(dirname "$0")" && pwd)"
if [[ "${1:-}" == "--qt" ]]; then
  shift
  exec bash "$LAB_TEST_DIR/start_general_debug_lab_qt.sh" "$@"
fi
exec bash "$LAB_TEST_DIR/start_general_perception_studio.sh" "$@"
