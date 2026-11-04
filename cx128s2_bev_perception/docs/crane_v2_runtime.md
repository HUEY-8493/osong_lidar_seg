# Crane V2 runtime

The normal build now builds and installs `crane_v2_experimental_node` and its shared `crane_v2_core` library. The executable name is retained for compatibility; it is the selected Crane runtime.

Build these packages in the ROS workspace that the operating container actually sources:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cx128s2_bev_perception lidar_bringup guardian_lidar_bridge
source install/setup.bash
```

Run the usual Crane profile:

```bash
ros2 launch lidar_bringup lidar_bringup.launch.py system_mode:=crane
```

This selects one V2 perception process, the Crane static TF profile and Crane Bridge. General (`system_mode:=general`) continues to select the existing general/legacy executable. For manual legacy recovery use `system_mode:=crane crane_engine:=legacy`.

The Guardian Manager helper already passes `system_mode:=crane`; after updating and rebuilding these packages it automatically selects V2 on its next bringup. No new Manager engine selector or Redis keys are needed. A running old process does not switch engines when files are rebuilt. Start/restart the operating service only when ready to apply. Host-side `guardian_manager.py`, `mobile_crane_status.py`, `cx128s2_control.py`, and `run_lidar.sh` must also be the current source versions; a container build does not update host-side Python copies. The existing `workspace_setup/scripts/setup_guardian_manager_service.sh crane` installer updates these copies and the service (it restarts the service, so this is an operator deployment step).

Direct `cx128s2_bev_perception.launch.py` defaults to `perception_engine:=auto`: a YAML `perception_mode: crane_beam_work` selects V2, general YAML selects legacy. Explicit `perception_engine:=v2` and `legacy` are available.

## Defaults and calibration

`config/crane_v2.yaml` provides the default 2-second length support and 3-second sudden-shrink grace. It is loaded after the base YAML and before an optional user overlay. Base YAML input_topic and use_sim_time, plus launch input_topic/target_frame overrides, still apply. The supported target frame is `world` because the validated installation geometry is world-calibrated; another frame is rejected rather than silently ignored. The first successful recorded/live TF is frozen as in the validated V2 baseline.

The legacy F2 Hook/Ground/clustering/motion/work_status threshold sections do not configure the new engine. V2 uses its validated R15+C4 configuration and explicitly declared startup parameters. The startup log identifies `Crane V2 R15+C4`, input topic and confirmation/hold times.

Current public FSM values: IDLE, LIFTING, SUSPENDED, LOWERING, LOWERING_CONFIRMATION. crane_moving defaults to background scene-histogram activity, without estimating speed or direction. It is independent of vertical FSM states. See `crane_v2_scene_motion.md`; the previous body_xy method is optional. Before a work-cycle length lock, IDLE ROS/Redis length follows the current observation. Once locked, temporary same-Cargo IDLE retains the largest established extent until confirmed Hook separation, Cargo loss or identity reset. Ground-contact completion alone does not shrink the retained extent. In SUSPENDED/LIFTING/LOWERING, Redis length is the maximum length collected in the initial 2-second work window, with connected larger observations allowed to extend it; actual fragment length remains separate. See `crane_v2_length_lock.md` for limitations, including potential underestimation under persistent partial visibility.

## Existing RViz

The V2 runtime publishes to the existing enabled `/crane_hook/markers` display. Orange spheres show virtual Hooks; green points are current selected Cargo Raw; the translucent box is the published extent, labeled `LOCKED ESTIMATED` once locked. Raw fragment length and DIRECT/HELD/NONE are shown separately. Held box geometry never generates green historical Raw. Markers expire after one second without updates. F2-only debug clouds/services are not implemented by V2.

The actual Sheave Holder outlines and DIRECT/HELD labels are separate from virtual Hook points. Current Holder XYZ clouds are available on `/crane_hook/left_body` and `/crane_hook/right_body`; the Crane RViz profile enables both. See `crane_v2_holder_display.md`.
