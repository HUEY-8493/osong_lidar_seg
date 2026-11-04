# workspace_setup

Docker-based ROS2 Jazzy workspace bootstrap package for Ubuntu 24.04 hosts.

This package is a ROS2 package, but its main purpose is host setup automation: installing Docker, preparing a mounted ROS2 workspace, building a persistent Jazzy image, and running a CycloneDDS-based container.

## Target Environment

- Host OS: Ubuntu 24.04
- ROS2 runtime: Docker container
- ROS2 distro: Jazzy
- RMW: CycloneDDS
- Host workspace: `~/ros2_jazzy_ws`
- Container workspace: `/root/ros2_ws`
- Compose directory: `~/docker/ros2_jazzy`
- Container name: `ros2_jazzy_lidar`
- Image name: `guardianai/ros2-jazzy-lidar:latest`

## Install Order

Run these commands from this package directory first:

```bash
cd ~/construct_ws/src/workspace_setup
./scripts/install_docker.sh
```

Log out/in, reboot, or run:

```bash
newgrp docker
```

Then configure the host socket buffers for CycloneDDS:

```bash
./scripts/setup_cyclonedds_sysctl.sh
```

## Optional APC UPS Setup

APC UPS support is intentionally separate from the workspace setup flow. To install
and configure `apcupsd` for a USB-connected APC/Schneider UPS, run it only when
needed:

```bash
./scripts/setup_apcupsd.sh
```

The script preserves the first-run backups at
`/etc/apcupsd/apcupsd.conf.workspace_setup.bak` and
`/etc/default/apcupsd.workspace_setup.bak`. A currently disconnected UPS is
reported as a communication warning; it does not make a successful installation
and service configuration fail.

Prepare the workspace and copy Docker assets:

```bash
./scripts/setup_workspace.sh --aliases
```

Build and start:

```bash
~/docker/ros2_jazzy/build_image.sh
~/docker/ros2_jazzy/start_ros2.sh
```

## Daily Usage

### Guardian Manager mode

Choose one mode when starting the manager. The mode selects the perception
configuration, bridge mode, Redis payload schema, and UPS behavior.

```bash
python3 scripts/guardian_manager.py crane
python3 scripts/guardian_manager.py general
```

When installing the systemd service, pass the same mode once. If the service
already exists, the installer updates changed Python sources and restarts the
service if needed, without registering it again. An explicit `crane`/`general` also updates the
existing startup wrapper mode and restarts when needed; GUI settings and the service
unit are preserved. The installer also updates `run_lidar.sh`, preserving the first
replaced runtime files as `*.setup-backup`. Use `--force` only to replace the service
configuration.

The selected mode is forwarded as `system_mode` to the common LiDAR launch, which
selects TF, perception YAML/Crane reference overlay, Bridge and the existing RViz
preset together. Existing RViz auto-start settings are unchanged. Direct launches
can use `ros2 launch lidar_bringup lidar_bringup.launch.py system_mode:=crane`
or `system_mode:=general`.

```bash
sudo ./scripts/setup_guardian_manager_service.sh crane
sudo ./scripts/setup_guardian_manager_service.sh general
sudo ./scripts/setup_guardian_manager_service.sh --force crane
```

Omitting the mode defaults to `general` for a new installation; an existing
startup wrapper keeps its mode unless one is explicitly requested. The existing `--mode crane` and
`--mode general` options remain supported for direct manager runs.

Start and enter the container:

```bash
rosjazzy
```

or:

```bash
~/docker/ros2_jazzy/start_ros2.sh
```

Enter an existing container:

```bash
~/docker/ros2_jazzy/enter_ros2.sh
```

Stop the container:

```bash
~/docker/ros2_jazzy/stop_ros2.sh
```

Check status:

```bash
~/docker/ros2_jazzy/status_ros2.sh
```

Rebuild the image:

```bash
~/docker/ros2_jazzy/build_image.sh
```

Rebuild without cache:

```bash
~/docker/ros2_jazzy/build_image.sh --no-cache
```

Remove only the container while keeping the image and workspace:

```bash
~/docker/ros2_jazzy/reset_ros2_container.sh
```

## Source Code and Build Workflow

Clone repositories on the host:

```bash
cd ~/ros2_jazzy_ws/src
git clone <repo>
```

Build inside Docker:

```bash
cd /root/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The container automatically sources:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///root/ros2_ws/cyclonedds.xml
```

## ROS_DOMAIN_ID

`ROS_DOMAIN_ID` is intentionally not hardcoded in the Dockerfile or compose file.

Set it inside the running container with:

```bash
~/docker/ros2_jazzy/set_domain_id.sh 10
```

Open a new container shell after changing it:

```bash
~/docker/ros2_jazzy/enter_ros2.sh
```

## RViz and GUI Notes

RViz inside Docker needs host X11 access. The compose file passes `DISPLAY`, mounts `/tmp/.X11-unix`, and mounts `~/.Xauthority`.

If `DISPLAY` is empty, the scripts only warn. ROS2 CLI still works.

SSH X11 forwarding, MobaXterm, and remote OpenGL setups can fail because of X11 authentication or OpenGL driver constraints. For stable visualization, installing and running RViz on the host Ubuntu system can be simpler:

```bash
sudo apt-get install ros-jazzy-rviz2
source /opt/ros/jazzy/setup.bash
rviz2
```

Use the same `ROS_DOMAIN_ID` and compatible DDS/RMW settings when visualizing host-side.

## Dockerfile vs Docker Compose

The Dockerfile defines what is permanently installed in the image. Packages such as `rviz2`, `colcon`, `rqt`, and `rmw_cyclonedds_cpp` belong there.

Docker Compose defines how the container is run: mounts, network mode, environment variables, privileges, and command. Compose changes can recreate the container, but they do not install packages into the image.

Anything installed manually with `apt install` inside a running container can disappear when the container is recreated. Permanent dependencies should be added to `docker/Dockerfile`, followed by:

```bash
~/docker/ros2_jazzy/build_image.sh
~/docker/ros2_jazzy/reset_ros2_container.sh --restart
```

## CycloneDDS Socket Buffer Fix

If you see an error like:

```text
failed to increase socket receive buffer size
```

raise socket buffer limits on the host, not inside Docker:

```bash
~/construct_ws/src/workspace_setup/scripts/setup_cyclonedds_sysctl.sh
```

This writes:

```text
/etc/sysctl.d/99-cyclonedds.conf
```

and applies it with:

```bash
sudo sysctl --system
```

## Humble/Gazebo Classic to Jazzy Notes

When receiving Humble or Gazebo Classic topics from Jazzy:

- Use the same `ROS_DOMAIN_ID`.
- Use the same or compatible RMW implementation when possible.
- Match QoS settings. `sensor_msgs/msg/PointCloud2` is often published as best effort.
- For CLI checks, try:

```bash
ros2 topic echo /points sensor_msgs/msg/PointCloud2 --qos-reliability best_effort
```

## Files

```text
workspace_setup/
  README.md
  docker/
    Dockerfile
    docker-compose.yml
    cyclonedds.xml
  scripts/
    install_docker.sh
    setup_apcupsd.sh
    setup_cyclonedds_sysctl.sh
    setup_workspace.sh
    build_image.sh
    start_ros2.sh
    enter_ros2.sh
    stop_ros2.sh
    status_ros2.sh
    reset_ros2_container.sh
    set_domain_id.sh
```

## External LiDAR control and collection GUI

`lidar_control_center.py` is a standalone desktop client. It does not replace,
patch, restart or roll back Guardian Manager. Both existing server APIs and
Crane commit `8009e8f` are supported. Start with monitoring only:

```bash
python3 /home/sjjin/construct_ws/src/workspace_setup/scripts/lidar_control_center.py \
  --general http://192.168.223.152:8765 \
  --crane http://192.168.223.151:8765 \
  --output ~/lidar_collections
```

Without `--output`, the client automatically creates `lidar_collections/general`
and `lidar_collections/crane` beside `lidar_control_center.py`, regardless of the
terminal's current directory. Copying the script to another computer carries this
default with it. `OUTPUT_DIRECTORY` near the top of the script can be edited, or
`--output` overrides it. The directory must be writable; installed system scripts
may need an explicit `--output`. This is the client download destination; the
existing recorder paths on General/Crane are unchanged.

The URLs are editable in the GUI. Verify the General IP for your deployment.
The app needs Python 3 and Tk (`sudo apt install python3-tk` if missing).
After building `workspace_setup`, it can also run as:

```bash
ros2 run workspace_setup lidar_control_center.py
```

For a preview with no real requests or commands:

```bash
python3 workspace_setup/scripts/lidar_control_center.py --demo --output /tmp/lidar_gui_demo
```

### Keyboard terminal dashboard (no Tkinter required)

The desktop uses Tkinter. If Tkinter or the display is unavailable, the client
automatically opens a `jtop`-style curses dashboard in an interactive terminal.
No pip GUI/TUI package is required; curses is part of the Linux Python standard
library. Explicit terminal mode and a safe, offline demo are available:

```bash
python3 workspace_setup/scripts/lidar_control_center.py --tui
python3 workspace_setup/scripts/lidar_control_center.py --tui --demo --output /tmp/lidar_tui_demo
```

Use a terminal at least 70 columns by 18 rows; 120 columns by 30 rows is preferred.
At 110 columns or wider, General and Crane appear as two bordered cards side by
side; narrower terminals stack them. Status dots, highlighted selection, yellow
prediction diamonds and a three-hour recognition progress bar make changes easy
to spot. `u` switches the decorative symbols/borders to ASCII if the terminal
font cannot display them. This changes presentation only.
SSH users need a TTY (`ssh -t`). `--headless` remains background monitoring with
no interactive screen. Both UIs use the same HTTP/SSH/ROS workers, file validation,
pending-job recovery and recording ownership guards.

| Key | Action |
| --- | --- |
| `Tab`, `g`, `c` | Select General or Crane |
| `1`–`5` | Status, tracking/prediction, files, logs, raw API/ROS |
| Up/Down / Page Up / Page Down | Select or scroll vertically |
| Left/Right | Scroll long reasons or JSON horizontally |
| Enter / Escape | Open full JSON details / close details |
| `n`, `s`, `r` | Normal, standby, recording (General 10s / Crane 60s); confirm with `y` |
| `a` | Toggle completed-file collection for selected computer |
| `b` | Toggle Crane 60s recording/download repeat; confirm enabling with `y` |
| `l`, `t` | Collect selected computer's SSH logs / tracking snapshot |
| `d`, `x` | General manual 60s SSH rosbag / three-hour recognition diagnostic automation |
| `i` | Enter known capture ID for lookup/recovery |
| `e` | Edit selected computer's HTTP URL and SSH address; Ctrl-U clears input |
| `?`, `q` | Help/status page / exit |
| `u` | Switch Unicode icons/borders to ASCII and back |

Stopping repeat is immediate. Exiting does not cancel an already-running remote
recording; its known ID is preserved for later collection. The terminal shows
freshness, unavailable fields, errors and the existing-coordinator conflict.
Predicted tracks are yellow; observations older than five seconds are dimmed.

### Controls and recording ownership

- Separate General/Crane cards show connection, PointCloud, perception, Docker,
  vehicle/beam summary, request ID, recording/download phase, and errors.
- NORMAL and STANDBY call the existing `/api/lidar/normal` and
  `/api/lidar/standby`. Manual recording uses the existing Capture POST (General
  10 seconds, Crane 60 seconds). Recorder readiness is not exposed by the old
  API, so healthy PointCloud does not prove the recorder service is ready.
- Crane repeat is opt-in: request 60 seconds, poll the returned ID, wait for actual
  archive availability, download, then request again. Stop-repeat disables new
  requests immediately; a current remote recording still finishes and is collected.
- General's detection-triggered 10-second recordings stay in the existing recorder
  package. The client can automatically download completed records when the
  installed Manager exposes its capture list. Legacy Managers lack that list;
  the client can poll/download known IDs and recordings it started itself. For
  General detection recordings unregistered in a legacy Manager, optional SSH
  automatically reads closed sessions under `/root/ros2_ws/cx128s2_dataset` and
  archives them without changing the server code. It requires existing key/Docker
  access and keeps only validated closed-session files in the ready list.
- The current General Manager still contains a Crane coordinator. The client detects
  its nonempty `crane_capture.manager_url` and blocks another Crane recording owner.
  Use the GUI for monitoring now. Before later enabling the external repeat, disable
  the previous coordinator (for example its existing `GUARDIAN_CRANE_MANAGER_URL`
  environment value can be empty) or perform your planned Manager rollback. No
  such live change is performed by this app.
- NORMAL/STANDBY require an explicit button action. Monitoring does not implicitly
  record. An optional three-hour SSH diagnostic also requires its GUI switch.

CLI monitoring and explicit external repeat are available without a display:

```bash
python3 workspace_setup/scripts/lidar_control_center.py --headless
python3 workspace_setup/scripts/lidar_control_center.py --headless --auto-crane
# When the old General has no list API, existing SSH can collect its recorder sessions:
python3 workspace_setup/scripts/lidar_control_center.py --headless --collect-general --general-ssh user@GENERAL_IP
```

### Collected data and readable views

```text
lidar_collections/
  general/ or crane/
    YYYY-MM-DD/
      communication.jsonl       # timestamp, target, category, request/response/error
      status/*.json             # complete received HTTP status snapshots
    pending.json                # durable in-flight ID or ambiguous POST intent
    captures/CAPTURE_ID/
      original.tar.gz           # untouched downloaded source archive
      raw_1hz.tar.gz            # Crane only: first full raw PCD per source second
      download.json             # received job metadata and download size
      summary.json              # frame/object counts and coordinate frame
      inventory.json            # all archived file names and sizes
      frames.csv                # repaired legacy quoted-field formatting
      session.json              # original session metadata
      objects.json              # recorded annotation objects (up to 5000 for viewing)
    tracking_snapshots/*.json   # optional SSH one-shot ROS snapshot
    remote_logs/*.log           # optional SSH journalctl collection
    diagnostics/*.tar.gz        # optional SSH General rosbag
```

Crane still records/transfers all frames on the unchanged server. The client
preserves the original archive and provides a separate 1Hz raw archive; this does
not reduce Crane storage or network traffic. Raw PCD point bytes are preserved.
Captured annotations/previews remain in `original.tar.gz`. No remote archive is
extracted onto local paths. General files are kept at their original frame rate.

GUI tabs group tracking, captures, categorized communication/command/alarm logs,
and complete API responses. Double-click a live track or log entry to inspect
its full observed fields/request response without truncating the reason text. Stored object data can be opened from the capture
list; it is explicitly a recorded snapshot, not a live track. Files and logs have
no automatic retention policy; choose storage capacity and cleanup separately.

Known IDs survive client restart. An interrupted download never becomes ready
until its length and archive metadata validate. An ambiguous lost POST response
is paused rather than repeated because the old API has no idempotency/list
facility. The GUI's ID lookup can recover a confirmed server capture ID. Do not
clear `pending.json` until independently confirming the server's pending job.

### Live tracking / predicted state / reasons without changing the LiDAR code

HTTP exposes compact summaries, not complete tracks. It cannot reveal live track
IDs or decision reasons absent from the response. The GUI does not infer them.
Optional ROS subscriptions read existing production arrays, diagnostics strings,
and `/rosout` directly:

```bash
source /opt/ros/jazzy/setup.bash
source /home/sjjin/construct_ws/install/setup.bash
# Set ROS_DOMAIN_ID / DDS discovery to the intended computer first.
python3 workspace_setup/scripts/lidar_control_center.py --ros-target general
```

`--ros-prefix` selects an existing namespace when configured. The subscription
uses `/detected_objects`, `/tracked_objects`, `/object_motion_states`,
`/road_vehicle_tracks`, `/vehicle_identity_memory`, `/debug/track_table`, and
`/debug/perception_diagnostics_table`; `/rosout` supplies ROS logs. Message types
must be installed/sourced on the GUI machine. HTTP connectivity alone does not
provide DDS connectivity, and identical domains/topic names on multiple machines
can mix sources. Configure domains/namespaces before labeling observations.

Tracks show ID, class, lifecycle, exact `TRACKED_PREDICTED` state, motion, distance,
speed, lost frames, decision/rejection reasons. For Crane ROS observation,
`/crane_beam/status`, diagnostics and command-status messages are also preserved
in the raw-details view and logs. Unknown fields are marked missing,
and older-than-five-second tracks are gray. The live view updates on receipt;
track arrays are saved at most once per topic per second. Raw ROS messages do not
prove hardware connectivity when using bag playback, so their source is labeled.

### Optional SSH server logs / three-hour rosbag

There is no journalctl or rosbag-start endpoint on the legacy HTTP server. If SSH
already works, configure `user@host` in the GUI or via `--general-ssh` and
`--crane-ssh`. SSH uses existing keys with BatchMode; passwords are not stored.
The tracking-snapshot button reads `/tracked_objects` on General or
`/crane_beam/status` on Crane through its existing container CLI. PyYAML
(`python3-yaml`) parses that one-shot response; no DDS connection from the GUI
machine is required. A snapshot is labeled as historical and becomes gray after
five seconds. The log button retrieves the last hour of `guardian-manager.service` (up to 2000
lines). The user needs journal/Docker permissions; errors are shown in the GUI.

General's three-hour continuous vehicle-recognition interval is shown and logged. It requires a positive existing Bridge `vehicle_count`,
fresh perception, PointCloud and packet health. Zero/missing vehicle count,
stale perception or unhealthy communication resets the interval. This uses the
existing vehicle summary; it does not assert a single unchanged vehicle ID. If the
current Manager exposes `operation_elapsed_sec`, its existing alarm/rosbag owner
is retained and no duplicate SSH bag is started. Its diagnostic capture can be
collected using the standard download path.

For an older General Manager without that feature, the explicit "3시간 진단 자동
(SSH)" switch can record approximately 60 seconds using the existing container
CLI and then download a compressed bag via SSH. The manual SSH bag button is
also available when no Manager owns that diagnostic flow. Topics are the verified
raw PointCloud, four object arrays, `/tf`, and `/tf_static`; absent Odometry is
not invented. Container/ROS/MCAP availability, message counts, normal termination,
and disk space are checked. Repeated diagnostic work in one client is blocked.
Only one external controller should own automation. SSH reconnect/process failure
and GUI closing during a bag need field validation; remote recording is bounded
by a 60-second timeout and remains on the server for recovery. Client restart
resets its own three-hour interval; a failed or excessively delayed status
observation also resets that interval; the client does not modify server boot policy.

### Validation

```bash
python3 -B -m unittest discover -s workspace_setup/test -v
```

Tests use localhost fake Managers and synthetic archives. They verify legacy
compatibility, actual archive readiness, coordinator/mode guards, short transfers,
restart checkpoints, ambiguous requests, safe archives, 1Hz selection, command
failures, three-hour alerts, and optional SSH paths without real LiDAR commands.
