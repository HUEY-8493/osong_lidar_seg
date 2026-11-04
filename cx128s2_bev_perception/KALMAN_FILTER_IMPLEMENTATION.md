# Linear Kalman Filter Phase-1 implementation

## Current tracking architecture

`processCloudFrame()` produces frame-local clusters and `ObjectFeature` centroids.
`updateTracks()` greedily associates each feature against existing tracks using the
existing distance/shape-memory logic. Before association, the legacy track state
is predicted using its velocity; after association it calculates a centroid-based
velocity and applies the existing `0.45 * old + 0.55 * measurement` EMA. Tracks
are created with monotonically increasing `next_track_id_`; IDs are not reused in
the current process. Lost tracks retain the existing range-dependent lifetime.

## Implemented KF

`LinearKalmanFilter` is a per-`Track` constant-velocity linear KF with state
`[x,y,vx,vy]`, position measurement `[x,y]`, and the requested F/H model. Q is
the white-acceleration model with `q = kf_process_accel_std_mps2^2`:

```
Q = q [[dt^4/4,0,dt^3/2,0], [0,dt^4/4,0,dt^3/2],
       [dt^3/2,0,dt^2,0], [0,dt^3/2,0,dt^2]]
```

R is isotropic position variance. New tracks initialize position from centroid,
velocity to zero, small position covariance, and deliberately large velocity
covariance. The filter uses header-stamp `dt`; only KF integration is clamped to
the configured min/max interval. Invalid/nonpositive `dt` does not predict.

Most importantly, Phase 1 keeps legacy `Track::x/y/vx/vy` as the association and
shape-memory state. KF state is separate and is predict/update only *after* the
existing association decision. In `kalman` mode, published objects and object
markers use the KF state; no EMA is applied to that output. A lost track gets
predict-only KF updates but its existing deletion policy is unchanged. Erasing a
Track erases its KF state with it.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `tracking_filter_mode` | `kalman` | `kalman` selects KF output; set `legacy` for A/B rollback |
| `kf_process_accel_std_mps2` | 1.5 | Acceleration-noise standard deviation |
| `kf_measurement_position_std_m` | 0.10 | Cluster-centroid measurement standard deviation |
| `kf_initial_position_std_m` | 0.10 | Initial position uncertainty |
| `kf_initial_velocity_std_mps` | 1.0 | Initial velocity uncertainty |
| `kf_min_dt_sec` / `kf_max_dt_sec` | 0.01 / 0.5 | KF integration clamp |
| `kf_debug` | false | Per-update innovation/state diagnostic logs |

Enable only for A/B replay:

```bash
ros2 launch cx128s2_bev_perception cx128s2_bev_perception.launch.py \
  params_file:=... # set tracking_filter_mode: kalman in the YAML
```

## Synthetic benchmark

`scripts/kalman_filter_benchmark.py` is deterministic (0.10 m Gaussian centroid
noise; Q/R defaults above). Its results are a mathematical regression benchmark,
not a substitute for identical-rosbag safety validation.

| Scenario | Legacy position RMSE | KF position RMSE | Legacy velocity RMSE | KF velocity RMSE | Winner |
|---|---:|---:|---:|---:|---|
| Static | 0.1358 | 0.0718 | 1.1764 | 0.2003 | KF |
| Constant velocity | 0.1358 | 0.0724 | 1.2781 | 0.2530 | KF |
| Sudden motion | 0.1358 | 0.0802 | 1.2453 | 0.3076 | KF |
| Sudden stop | 0.1358 | 0.0719 | 1.2176 | 0.3215 | KF |
| One-frame dropout | 0.1358 | 0.0722 | 1.3053 | 0.2531 | KF |
| Three-frame dropout | 0.1358 | 0.0758 | 1.2637 | 0.2552 | KF |
| Irregular dt | 0.1358 | 0.0732 | 1.2905 | 0.2564 | KF |
| ID recreation | state reset verified | 0.0724 | state reset verified | 0.2530 | KF lifecycle safe |

The script additionally reports 1/10/50/100-filter Python execution scaling.
Measure C++ `tracking_ms` p50/p95/p99 on a real rosbag before changing defaults.

## Verification and conclusion

## rosbag A/B metrics (2026-07-29 bag)

Both modes replayed the same 312.5-second `/ch/lslidar_point_cloud` input at
1x after explicitly loading the current static-background file. One persistent
track was present in nearly every output frame. The bag has no ground truth, so
the position and velocity values below are temporal-noise proxies, not accuracy
RMSE: position is the RMS consecutive XY displacement and velocity is the RMS
consecutive velocity-vector change for the same Track ID.

| Metric | Legacy EMA | Linear KF | Change |
|---|---:|---:|---:|
| Output frames / non-empty frames | 2924 / 2920 | 2944 / 2940 | comparable |
| Track IDs | 1 | 1 | no ID switch observed |
| Position step RMS [m] | 0.03274 | 0.01239 | **-62.2%** |
| Position step p95 [m] | 0.05527 | 0.02129 | **-61.5%** |
| Velocity delta RMS [m/s] | 0.31196 | 0.03704 | **-88.1%** |
| `tracking_ms` p50 / p95 | 0.031 / 0.048 | 0.039 / 0.048 | +0.008 ms p50; p95 unchanged |
| `total_pipeline_ms` p50 / p95 | 106.501 / 114.914 | 105.529 / 112.318 | no material regression |
| Drop-ratio p50 | 0.061 | 0.058 | comparable |

The observed object is sufficiently stable for a smoothing/noise comparison,
but the bag does not establish moving-object position RMSE, TTC error, response
lag, or stop overshoot. Those require ground truth or a separately annotated
moving-object segment before changing the production default.

- `colcon build --packages-select cx128s2_bev_perception`: passed.
- `ctest ... -R test_linear_kalman_filter`: passed (initialization, predict,
  update, variable `dt`, invalid `dt`, predict-only dropout, and recreation).
- `python3 scripts/kalman_filter_benchmark.py`: passed for S01--S08.

Conclusion: **B — KF is applicable, with real-data Q/R tuning still required.**
Keep `legacy` as default until the same rosbag shows acceptable sudden-motion,
stop, and lost-track behavior. Phase 2 (KF prediction for association): **NO for
now**; consider it only after Phase-1 A/B data proves output-quality benefit and
association/ID-switch metrics are separately evaluated.
