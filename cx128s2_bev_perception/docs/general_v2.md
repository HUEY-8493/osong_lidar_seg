> 현재 상태(2026-11-04): 사용자 요청으로 기본 실행은 기존 General Legacy로 복귀했다. V2는 명시적 `perception_engine:=general_v2` / `--engine v2` 실험 경로로 보존하며, 아래 내용 중 기본 선택에 관한 과거 기술보다 이 안내가 우선한다. 신규 V2 후보 생성 단계의 지면 노이즈 제외는 10/26 실제 Bag 59프레임으로 검증했다. 10/29는 이번 변경판 전체 Replay를 아직 수행하지 않았다.

# General V2 — independent perception engine

2026-11-04. This replaces the earlier source-file extraction as the new General development engine. It does **not** include or call the legacy node, `src/general/*.inc.hpp`, Crane V2, legacy background subtraction, FEM memory or legacy tracking. The old engine remains selectable for comparison/recovery. Existing uncommitted Crane code and the old General detector are preserved.

## Implemented pipeline

| Stage | General V2 behavior |
|---|---|
| Input | Validate organized/unorganized FLOAT32/FLOAT64 XYZ, row padding, endianness and finite coordinates; transform at the original sensor timestamp. |
| Coordinates | World/output-frame XYZ. Crop distance and public `distance` remain planar distance from the transformed sensor origin. Entry/exit direction uses world-origin range, preserving the existing status helper's sign convention. |
| Ground | Current-frame tiled lower envelope, deterministic RANSAC and supported plane fit. No static map or background learning. Missing/degenerate support is UNKNOWN, never an assumed z=0 floor. |
| Height | Points **>= 0.20 m vertically above the observed plane** and below the configured upper height limit enter detection. Ground estimation can read below the legacy obstacle `valid_z_min`. Raw vehicle geometry cannot reintroduce discarded ground points. |
| ROI | Existing watch/ignore/hard_ignore policy; package://, /filename and bare-filename path conventions; frame validation; reload/clear/save services and ROI markers. Zero-area watch remnants are disabled; invalid exclusion geometry is rejected. |
| Detection | Native candidate admission excludes corroborated low-ring raised-floor artifacts **before** any component, box or track is created. Optional ring is read from Raw; missing/unsupported beams or missing local floor support pass through. Nearby observed upper body protects low body/wheel returns. No Legacy filter or node is called. Sparse XY grid, height-overlap-aware connected components, exact associated current Raw indices, measured PCA-oriented box and configurable physical vehicle geometry checks. Oversized/ambiguous geometry remains UNKNOWN rather than being arbitrarily promoted/split into vehicles. |
| Confirmation | Object and vehicle geometry confirmation are separate. Default >=3 observations and >=0.5 sensor seconds; an old UNKNOWN track's age cannot confirm one new vehicle-like frame. Stationary vehicles can confirm without motion or prior work history. |
| Tracking | One-to-one global Hungarian association, bounded position/shape gates and constant-velocity filtering. No nearest-large-object unconditional identity merge. |
| Partial observations | Current Raw dimensions/indices remain separate. Switching to a short fragment does not create displacement. Repeated same-shape fragments can provide measured translation while the full body pose/extent is estimated. Unsupported partial motion does not add stopping/directional evidence. |
| Dimensions | Default 2 s observation before retaining the largest supported same-track dimensions; retained dimensions do not shrink during that identity. Growth requires independent temporal support (0.3 s). A new identity starts fresh. Retained geometry is marked separately in evidence/decision_reason. |
| Loss | DIRECT requires current Raw. HELD contains zero current body points and zero observed dimensions. Public hold expires after 1.3 sensor seconds. Only confirmed stopped vehicles have an additional conservative, hidden 10 s identity recovery window. |
| Motion/status | Observed motion window and hysteresis; stopped dwell remains 10 s by default. Missing/unsupported evidence pauses dwell rather than manufacturing stationary time. Public vehicle statuses remain `entering`, `stopping`, `stopped`, `exiting`. Away from world origin = entering; toward origin = exiting. |
| Sensor time | Source header stamp reaches all public arrays/recording output. Zero/invalid and duplicate time cannot advance evidence; regression starts fresh temporal tracking. No PC-clock sensor timestamp substitution. |
| Execution | ROS adapter, standalone core and synchronous offline replay share the same engine. Live mode uses a bounded latest-frame queue; offline replay processes every source cloud. |

Source: `include/cx128s2_bev_perception/general_v2/core.hpp` and `src/general_v2/{ground,detector,tracker,core}.cpp`. ROS input/output lives in `ros_adapter.hpp`; `node.cpp` owns the live worker and `replay.cpp` owns offline input/result persistence.

## ROS / Manager / Redis compatibility

The `TrackedObject`/array and recording messages are unchanged. The existing five topics are published: `/detected_objects`, `/tracked_objects`, `/object_motion_states`, `/road_vehicle_tracks`, `/vehicle_identity_memory`. Confirmed physical outputs use TRACKED/TRACKED_PREDICTED; identity output uses ACTIVE/TRACKED_PREDICTED. Motion and radial strings retain the existing vocabularies. Positions and velocities are in the output frame, length is `dimensions.x`, dimensions/distances are meters. HELD/partial/retained evidence appears in existing `decision_reason` and offline internal fields, not new Redis keys.

The actual Bridge `FirstVehicleSelector` and `general_summary` remain unchanged: choose the **longest** current vehicle and take its length, distance and status together. Header sensor time remains compatible with `sensor_timestamp_ms`. The existing General Redis payload-generation timestamp policy itself is unchanged. Controlled projection tests do not claim an actual running Manager/Redis connection; no live Redis write, service restart or deployment was performed.

`/background_status` reports BACKGROUND_DISABLED, `/object_status`, object/ROI markers and optional recording/debug clouds are available. Static-background services return explicit unsupported responses. Legacy FEM/class-memory commit, hand-authored shape graphs, FIFO permanent identity and background-learning workflows are not part of this new engine; they are intentionally not silently run in parallel. `/load_roi`, `/clear_roi`, `/save_roi` operate on the active policy. Parameters are startup-only (except use_sim_time); change YAML and restart the node to retune. ROI file edits can be reloaded without restarting. A configuration change must not silently keep incompatible history.

## Selection and build

`perception_engine=auto` selects **Legacy for General**, and Crane V2 for Crane YAML.
Select the new engine explicitly with `perception_engine:=general_v2`, or bringup
`general_engine:=v2`. Existing General default and Crane selectors are preserved.
No production service has been restarted.

```bash
cd /home/sjjin/construct_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to cx128s2_bev_perception --cmake-args -DBUILD_TESTING=ON
source install/setup.bash

# Offline review of independent V2, latest TWO compatible bags.
python3 src/cx128s2_bev_perception/tools/general_batch_review.py \
  --engine v2 \
  --results "$HOME/general_v2_review_$(date +%Y%m%d_%H%M%S)"
```

No background-map argument is needed. Existing General YAML provides crop/vehicle geometry/ROI calibration; `config/general_v2.yaml` supplies independent engine defaults, then an explicit launch overlay can override them. Legacy static-map/FEM/cluster/KF parameters do not configure the new engine. New parameters are listed in `general_v2.yaml`; ground seed height bounds are in the output frame and must match the site's geometric calibration, not a learned background.

The two default bags are currently `rosbag2_2026_10_26-02_12_32` and `rosbag2_2026_10_29-07_28_08`. `--bag NAME` is repeatable, `--limit N` deliberately changes the default maximum, and `--list-only` lists selected inputs without copying/replay. Omitting `--engine` runs the existing Legacy default; `--engine v2` is required for the new engine. A new results directory is required. A compiled source fingerprint prevents running stale V2 binaries; use `--build-dir` for an isolated package build directory.

Results include per-frame JSONL/CSV, sensor timestamps, observed and retained geometry, ground validity, confirmation, status, timing Mean/P99/Max, timeline PNG, current-Raw BEV/XZ/YZ representative PNGs, contact sheet, and static HTML. Green points are actual associated current Raw, gray other current Raw, brown ground-filtered Raw; dashed blue is retained/predicted geometry. A common cropped view is used without point decimation. Images and algorithm labels are evidence, not independent ground truth. Up to 32 representative frames mix events and periodic coverage; the CSV/JSONL covers every frame.

Only the runner's newly allocated local Bag copy and temporary preview binaries are removed in normal/failure/interruption cleanup. NAS originals are read-only inputs. SIGKILL/power loss can leave an owned scratch directory; it is never confused with a user-supplied source. Do not run the offline tool alongside hardware services on its isolated test ROS domain.

## Validation and remaining uncertainty

- Standalone core tests cover floor/slope/UNKNOWN, inclusive 0.20 m boundary, stationary startup and reboot, independent vehicle confirmation, full/partial/moving-partial dimensions and translation, growth, bounded loss/HELD, global assignment/multiple IDs, entry/exit, ROI and sensor-clock rejection/reset.
- ROS adapter tests cover actual PointCloud2 decoding, padded rows, malformed input, preserved sensor stamp, static/missing TF and translated sensor-origin distance, public message units/status/motion vocabularies.
- `test/smoke_general_v2_batch.py` creates **synthetic-only** temporary bags and exercises the actual batch runner, source fingerprint, two-bag count, Raw replay, images, original-fixture preservation, cleanup and unchanged Bridge projection. It never accesses a field/NAS bag. AddressSanitizer/UndefinedBehaviorSanitizer testing was also run on the standalone core; details are in the validation artifact directory.
- Current native admission change was tested with actual 10/26 Raw+TF, all 59 frames: public vehicle presence 0 → 47 frames, first confirmed frame 13. The overlong noise-linked F32 component becomes a roughly 17.85 m vehicle observation. Mean pipeline 23.24 ms, maximum 26.76 ms on this machine. These results do not establish independent ground truth, general field accuracy, or long-duration performance. 10/29 has not been replayed in full with this change.
- Remaining risks: low/poorly spread ground samples, wrong ground seed calibration, nonplanar roads, touching vehicles/nearby structures fusing into one component, geometry-only false positives, sparse long-range fragments, heading flips on nearly square observations, and overlarge retained dimensions after real contamination. Exact-time dynamic TF absent at cloud time is an error, not a silent latest-transform fallback. These require the user's two-bag review before a field-quality claim or further tuning.

Validation artifacts: `/home/sjjin/crane_version_review/general_v2_20261104`. Synthetic reviews are clearly named `rosbag2_2099_*` and must not be presented as real measurements from the entrance recordings.


## Native candidate admission (2026-11-04)

`general_v2.candidate_ground_noise_rejection: true` is built into candidate creation.
It does not postprocess vehicle outputs or reuse the Legacy noise filter. Only current
Raw points within the active crop/ROI contribute. A 0.5 m cell needs >=20 actual near-plane
floor returns in >=4 beams, q90-q10 <=0.12 m. A separate layer 0.18–0.8 m above the
measured local floor is excluded only with >=3 points, <=4 beams, >=75% in the two
most populated beams, and layer/floor count <=0.35. Nearby (1 m) upper-body support
of >=6 points in >=3 beams at 0.9–5 m above the observed plane protects the cell.
There are no fixed ring numbers, scene XY masks, static backgrounds or assumed Z=0 floors.

The optional Raw `ring` field accepts UINT8/UINT16 including big-endian/padded layouts;
missing/malformed fields are UNKNOWN. UNKNOWN beam evidence cannot authorize exclusion.
Excluded candidates remain separate from measured ground. Offline trace and internal
V2 diagnostics count them as `candidate_excluded_points`; no ROS message or Redis key
is added. Debug candidate foreground omits them; original Raw remains unchanged.
Review images mark excluded points orange and measured ground brown.

A low real isolated object can still resemble a sparse artifact. Body protection and
floor/beam corroboration reduce that ambiguity but do not replace independent ground truth.
Artifacts: `/home/sjjin/crane_version_review/general_v2_candidate_noise_20261104/`.
