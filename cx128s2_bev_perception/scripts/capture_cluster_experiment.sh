#!/usr/bin/env bash
# Capture one reproducible BEV clustering experiment and print its key metrics.
# Usage: capture_cluster_experiment.sh [label] [seconds]
set -euo pipefail

label="${1:-z_profile_probe}"
seconds="${2:-60}"
out_dir="/tmp/cx128s2_cluster_experiments"
mkdir -p "$out_dir"
stamp="$(date +%Y%m%d_%H%M%S)"
log="$out_dir/${stamp}_${label}.txt"

echo "[$label] Capturing ${seconds}s -> $log"
timeout "${seconds}s" ros2 topic echo --no-daemon --full-length --spin-time 3 \
  /debug/perception_diagnostics_table \
  | perl -pe 'if (s/^data: "//) { s/"$//; s/\\n/\n/g; $_ .= "\n---\n"; }' > "$log" || true

frames="$(awk '/^timestamp:/{n++} END{print n+0}' "$log")"
echo "frames=$frames"
for key in raw_component_count valid_cluster_count tiny_component_count \
  largest_component_cells largest_component_points clustering_ms total_pipeline_ms \
  z_profile_cardinal_tolerant_overlap_mean z_profile_diagonal_tolerant_overlap_mean \
  z_profile_bridge_tolerant_overlap_mean; do
  awk -F: -v key="$key" '$1 == key {gsub(/^[[:space:]]+/, "", $2); sum+=$2; if (!n || $2 < min) min=$2; if (!n || $2 > max) max=$2; n++}
    END {if (n) printf "%s mean=%.3f min=%.3f max=%.3f n=%d\n", key, sum/n, min, max, n}' "$log"
done
echo "log=$log"
