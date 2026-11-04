# General Legacy: corroborated low-return artifact filter

Enabled by default in `cx128s2_bev_perception.yaml` as `general_ground_noise.enabled`.
The default General engine remains Legacy; this does not enable experimental General V2.
`valid_z_min=0.08`, ground estimation input, and the 0.20 m foreground margin remain unchanged.

The filter runs after ground/background classification and before outlier removal and clustering.
Only current-frame Raw XYZ and ring measurements are used. No static background or historical
detection output is needed. Ground fitting receives its original input.

A 0.5 m XY cell needs at least 20 actual floor points in at least four rings, within
0.15 m of the configured world-ground reference (default 0 m), with q90-q10 <= 0.12 m.
A layer 0.18–0.80 m above that measured median is a candidate only with at least three
points, at most four rings, at least 75% in the two most populated rings, and a layer/floor
point ratio <= 0.35. A current upper-body cell containing at least six points in three
rings at 0.9–5.0 m protects its 1 m XY neighborhood. Ring numbers are never blacklisted.

Candidate points are excluded from vehicle foreground and Raw dimension refinement.
They are **not** relabelled as observed ground. The original Raw debug cloud remains Raw
and can still display those returns. Missing ring, unsupported ring >=256, or missing floor
support passes through. Parameters under `general_ground_noise.*` require a node restart.
All parameter defaults and validation are in `general_ground_noise_filter.hpp`.

This is deliberately conservative. An isolated low object with sparse concentrated beam returns
can resemble an artifact; areas without corroborating floor evidence are not filtered.
The ground reference is a calibration hint, not a substitute for actual measured floor points.
Crane paths and public ROS/Redis schemas are unaffected.

Build normally:

```bash
cd /home/sjjin/construct_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to cx128s2_bev_perception
source install/setup.bash
```

Validation artifacts: `/home/sjjin/crane_version_review/general_noise_filter_20261104/`.
