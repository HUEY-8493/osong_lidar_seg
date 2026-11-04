> Current default (2026-11-04): original General Legacy is restored. General V2 is preserved as an explicit experiment. The batch tool defaults to legacy. The valid_z_min 0.08/0.33 field comparison is recorded at `/home/sjjin/crane_version_review/general_legacy_z033_20261104/index.html`.

# General modular refactor and local batch replay

The basic processing stages are split into `src/general/*.inc.hpp`: data types, configuration, ROS/TF input adapter, BEV clustering, alternate cluster plugins, ground/foreground processing, vehicle feature/Raw-box refinement, frame pipeline, tracking/lifecycle and publication. They remain members/types of the same translation unit; shared node state and the legacy Crane adapter remain in place. This is a modular source refactor, not a new standalone detection engine.

General now defaults to **ground_only**, without static background loading, subtraction or learning. Points at least **0.20 m above estimated local ground** enter obstacle detection (subject to the existing upper height limit, ROI and validity filters). Near/mid/long ground margins and obstacle minimum height are all 0.20 m. `use_fixed_ground_plane` remains false: the criterion is relative to the estimated ground, not simply world Z >= 0.20. The ground/obstacle boundary includes exactly 0.20 m. Ground estimation itself, vehicle tracking/lifecycle, public messages, longest-vehicle Redis selection and Crane V2 are unchanged. Ground-only Raw-box refinement also uses only ground-filtered points, so it cannot reintroduce removed floor points. Static-mode Raw refinement is retained. Expanded source matches the pre-refactor source except for the two ground-boundary comparisons, this Raw input selection, and one whitespace cleanup.

No real Bag replay, deployment or Redis writes were performed during this change. User review of batch results is required before accuracy claims.

## Build and run

From a sourced ROS workspace, retain testing targets so the offline build script can reuse the package's exact compiler/link flags:

```bash
cd /home/sjjin/construct_ws
colcon build --packages-up-to cx128s2_bev_perception --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
python3 src/cx128s2_bev_perception/tools/general_batch_review.py \
  --results "$HOME/general_review_$(date +%Y%m%d_%H%M%S)"
```

No background map argument is needed. The batch runner defaults to `ground_only`, and the General production YAML uses the same mode and 0.20 m policy. Existing ROI/visibility calibration still applies. Do not deploy/restart a service just to run this offline tool.

The default maximum is **two latest compatible bags**, processed sequentially:

- `rosbag2_2026_10_26-02_12_32`
- `rosbag2_2026_10_29-07_28_08`

`--list-only` prints the selected two without copying/replaying. `--bag NAME` selects specific bags; repeat it to select two explicitly. `--limit N` deliberately changes the maximum (must be positive). `--build-dir /path/to/build/cx128s2_bev_perception` supports a nonstandard build directory. Run from the source checkout (the tool depends on source/test harness files).

Offline harness prerequisites in addition to normal package dependencies: `rosbag2_cpp`, `tf2_msgs`, OpenSSL headers, LZ4 headers, xxhash runtime, Python PyYAML. The batch tool uses `--full-only`, so optional AST/cache fingerprint Python dependencies are not required. The build log records dependency failures. The host's ROS distribution/build flags must agree; the current inherited harness build script targets ROS Jazzy.

## Ownership and results

Default source: `/mnt/z/osong/oosong_record/enterance`. Default scratch: `$HOME/.cache/general_bag_review`. Only metadata and its explicitly listed storage files are copied, sequentially, into a newly allocated owned temporary directory. Source symlinks/path traversal and network-backed scratch are rejected. Copy hashes, source size/mtime stability and free space are recorded/checked. Original NAS data is never deleted. Normal completion, subprocess failure and handled interruption clean the owned temporary directory; a machine crash/SIGKILL can leave a directory named `owned-general-*` for manual cleanup.

Each run requires a new results directory. Results contain build/source provenance, per-bag copy manifest, effective parameters, command/log, per-frame sensor stamps and vehicle dimensions/status/positions, tracking CSV, pipeline timing CSV, summary JSON and `review_index.html`. Effective parameter records include temporary input paths for provenance; these files no longer exist after cleanup. Detector thresholds are not adjusted automatically.

The full synchronous harness reads original Raw plus recorded TF and calls the production pipeline directly; there is no latest-only ROS queue, detector-result input, Manager, Redis writer or sensor driver. DDS discovery is localhost-only on domain 173; the tool does not run an executor. Missing/invalid TF, nonmonotonic sensor stamps and frame-count mismatch are failures. Missing output frames are reported separately from input frames. `COMPLETED_UNREVIEWED` means execution finished, not that detection is correct. Zero detections without independent ground truth is not a passing accuracy result.

This stage produces numerical review artifacts; automated Raw PointCloud image rendering is not included. Inspect the vehicle JSONL/CSV and return failing bag names, timestamps/frames and logs for the next refactor.
