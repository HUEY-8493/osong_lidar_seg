# Crane Perception V2 — experimental, not adopted

Status (2026-11-02, candidate R15): the independent implementation compiles and runs, but **fails physical perception acceptance**. The deployed/default Crane selection remains F2. Do not select this executable for a production service. Performance or sanitizer success does not establish correct recognition.

The review artifacts are in `/home/sjjin/crane_version_review/v2_engine_20261101`. `request.txt` preserves the original requirements; `f2_backup` includes the original source and Git metadata. The new engine never calls the F2 detector, BEV, ground, tracking, motion or FSM. General MC6, existing launch/YAML, messages and Bridge are unchanged. No installation, service restart or live Redis write is part of this work.

## Architecture

`Engine::process(world_points, sensor_stamp_ns)` is a ROS-independent, serial API. `Point::raw` preserves the original cloud index. Inputs and returned output own their memory; an output remains valid after the next call or reset. There is no global pool, worker, CUDA, previous-frame cloud cache or KF.

1. Keep finite points within the calibrated watch volume. Stable radix grouping builds active 3D voxel cells in contiguous vectors. A contiguous open-address lookup supplies neighbours without per-cell hash allocation. Cell graph and raw indices are shared across observations.
2. Observe compact Housing surfaces in the two installation ROIs; calculate Virtual Hook as measured Housing minus the configured 0.9m installation offset. A short absence retains a labelled held position without fabricating direct points or motion.
3. Build an independent current-frame floor height hypothesis from spatially distributed raw low surfaces. A locally supported constant-height plane is an **estimate under the Cargo**, not a directly visible occluded floor. Missing support is UNKNOWN. This prototype does not yet robustly distinguish raised surrounding structures from cargo/support surfaces.
4. Form raw-supported 3D Cargo components. Lateral support rejects some thin rigging; weighted plane evidence and a compact XY surface sheet distinguish panels from sparse outlines. A separate current-Raw prismatic cross-section hypothesis supports narrow beams whose visible surface is mostly vertical. A compact transverse profile separates repeated neighbouring surfaces when current measurements support that separation. These geometric hypotheses still truncate or merge touching surfaces in 10/24. Hook ROI is a seed relationship check, not a cargo length boundary.
5. Track by measured surface height, oriented footprint overlap, lateral distance and retained dimensions. Initial confirmation requires continuous strong direct evidence; the more permissive partial-observation path only applies to confirmed tracks. The elongated-body gate reduces parallel-object swaps, but the 10/24 full run still has 11 ID switches between initial unconfirmed candidates; all 666 confirmed direct observations keep ID 12. This numerical stability does not prove identity after physical contact with a neighbouring object.
6. Compare current raw body patches in a measured object-relative XY frame. Only matched direct Z change can create motion. No Hook delta, KF or held Z creates Cargo motion. If the footprint changes, independently measured planes must agree at the same world XY before accepting patch motion. A deterministic tilted stationary Cargo with progressive occlusion reproduced false LIFTING in R11 and passes in R12. The short 0.5s path requires three directional steps and 0.025m displacement. A separate 1.5s direct-surface path requires at least 1s duration, eight consistent steps and 0.018m displacement to retain slow measurable motion. Actual partial-object rotation and nonplanar surfaces remain insufficiently validated.
7. Emit the five existing public states. Confirmed direct airborne Cargo under the Hook can enter SUSPENDED without prior LIFTING. The settings preserve 0.10m clearance, ±0.15m Hook XY evidence, 2s initial confirmation, independent LOSS_TIMEOUT 1.3s / GROUND_CONTACT / HOOK_DETACHED with 0.3s contact/detachment checks, and a 3s completion event. Missing Hook/rigging pixels yield UNKNOWN, not separation. A previously confirmed work cycle is retained during uncertain connection evidence, while only new direct Cargo measurements can create motion. Current confirmed surface motion may enter LIFTING/LOWERING with Ground UNKNOWN; UNKNOWN does not prove contact or stationary clearance. An input gap restarts the continuous-input timer and cannot immediately prove Cargo loss. Duplicate/reversed/invalid sensor timestamps are rejected without updating event history.

`Config` documents installation/watch limits and physical thresholds. These are experimental defaults copied from installation/rule information, not newly edited production YAML. The existing learned detector/reference matcher is not called. The experimental automatic adapter is not a replacement for production manual command/control/watchdog handling.

## Build and test

From a ROS 2 Jazzy shell with the package dependencies available:

```bash
cmake -S /home/sjjin/construct_ws/src/cx128s2_bev_perception -B /tmp/crane-v2-build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build /tmp/crane-v2-build --target \
  crane_v2_experimental_node crane_v2_replay test_crane_v2 test_crane_v2_ros
ctest --test-dir /tmp/crane-v2-build -R '^test_crane_v2' --output-on-failure
```

The experimental Node is `EXCLUDE_FROM_ALL` and is not installed or selected by production launch. It subscribes to `/ch/lslidar_point_cloud`, receives actual TF using the same executor, freezes the initial sensor-to-world transform, and publishes the existing `/crane_beam/status` and `/crane_beam/working_status` contracts. Sensor stamps remain the original ROS header; no PC timestamp is substituted. It uses AUTO only. Always use a separate local ROS domain for experiments; do not run alongside production on its domain.

`test/crane_v2_replay.cpp` consumes the explicitly exported world-point cache. Per frame: little-endian `int64 stamp_ns`, `uint32 valid_point_count`, then that many 16-byte records `{float x,y,z; uint32 original_raw_index}`. The artifact directory's `export.cpp` reads actual PointCloud2/TF from the unmodified rosbag and writes these caches; `.transform.json` records the transform. `ros_replay.cpp` additionally runs directly from ROS bags for formal decode+core timing. Cache I/O, ROS deserialization and JSON recording are excluded from that timer; decoding/transformation and the full Core are included. Profiling, sanitizer and clean Release measurements are separate.

Tests cover stationary startup/restart, actual raw translation, XY/yaw changes, partial geometry, ownership separation, Hook-only/wire/ground negatives, unknown Ground, independent completion reasons, short loss vs 1.3s, 3s event/no repeat, input gaps, duplicate/reversed/zero timestamps, independent instances, reset, >400k input followed by small/empty input, output validity, ROS layout/row stride, schema and nanosecond stamps. Synthetic passing tests do not override failing actual bag results.

## Known blockers to adoption

The current four-bag R15 run has no public work on either negative (392 + 99 IDLE frames). Unconfirmed direct candidates still occur: 55 and 39 frames respectively; this is not a claim of zero detector false positives. New 10/30: IDLE 42, SUSPENDED 981, LIFTING 18, LOWERING 20, no completion; Cargo DIRECT 991/1061, one ID. This fixes the earlier false-work and false-detachment failures, but motion counts alone are not physical ground truth.

10/24: DIRECT 786/5006, SUSPENDED 527, LIFTING 5, LOWERING 11, one Ground Contact completion held F4844–4874. Initial unconfirmed candidates use 12 IDs; every confirmed direct observation (666 frames) keeps ID 12. The user confirmed that V2's elevated green object at F4395/F4440 is the actual Cargo and the other/F2 selection is not. F2's old completion interval is a comparison, not the required answer. Nearby surfaces can still merge near contact; final target identity and precise completion timing are not fully established by that two-frame annotation.

**Bottom policy:** the user explicitly allows an ESTIMATED underside from sufficient measured shape. `estimateBottom` requires repeated spatially supported top/wall/lower-edge evidence across at least three longitudinal sections, consistent vertical depth and length coverage. It weights occupied measured cross-section cells equally so dense top Raw cannot hide a sparse side. A tilted top sheet alone returns UNKNOWN. The inferred lower reference is always `ESTIMATED`, never DIRECT; the existing `beam_ground_clearance_reason` carries the ESTIMATED label without a new public message key. `beam_z_min/max` remain actual measured Raw bounds, independently of the estimated bottom.

Bottom evidence is sparse, especially for the broad 10/30 target. The FSM may retain an already confirmed work cycle while the current bottom/connection is UNKNOWN; such SUSPENDED frames are labelled `HOLD_CONFIRMED_LOAD_OBSERVATION_UNCERTAIN` internally and do not prove a current clearance measurement. Unknown current bottom does not generate Ground Contact. No retained depth is invented from a fixed Cargo thickness. External circle-fitting diagnostics were inconsistent and are not used in the engine.

Detailed candidate-specific timing, allocations, real 10Hz DDS transport, per-frame evidence, source hashes and visualizations are in `/home/sjjin/crane_version_review/v2_engine_20261101/REPORT.md`. Historical R4/R12 performance and DDS artifacts are separate from R15. No Jetson Orin NX measurement has been performed.

F2 remains the production/default engine pending full physical acceptance of contact-time target separation and underside estimation. The experimental adapter also lacks production manual/control integration. Do not remove F2 or change default launch on the basis of synthetic tests, more active-state frames or faster timings.
