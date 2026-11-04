# CX128S2 BEV data recorder

## General-only automation with unchanged Crane commit 8009e8f

Deploy these recording changes on **General only**. Crane remains on
`8009e8f5154ee60a96f8be4b6da90ad75b1b78be`; no Crane code update, new server,
new topic or new API is required. Crane's existing Manager must run in crane
mode and its existing recorder must already provide `/start_recording` and
`/stop_recording`. That commit's default bringup disables the recorder: having
its source installed alone does not mean the services are running. Existing
commands/configuration must start it if it is not already running.

General uses exactly the old HTTP endpoints:

```text
POST Crane /api/lidar/capture {"duration_sec":60}
  -> save the returned capture_id on General
GET  Crane /api/lidar/capture/ID until its archive is actually available
GET  Crane /api/lidar/capture/ID/download
  -> select one raw PCD per source timestamp second on General
  -> save the resulting archive locally
  -> issue the next 60-second request on the next poll
```

The default Crane URL is `http://192.168.223.151:8765`; override it with
`GUARDIAN_CRANE_MANAGER_URL`. An empty value disables remote requests. Polling
runs in a background thread every 10 seconds. Crane being offline does not block
General's vehicle recorder, UPS/Redis flow or main loop. Finalization, transfer
and polling introduce gaps between recordings.

**1Hz is applied to the received dataset on General.** The unchanged Crane still
writes and sends every sensor frame. General retains the first raw PCD in each
`stamp_sec` bucket, along with matching CSV/annotation/object files. Individual
PCD bytes and their original filenames are preserved. It repairs the legacy
CSV's unclosed final empty field when writing the filtered CSV. Session metadata
adds `general_save_interval_sec`, `general_original_raw_frames` and
`general_retained_raw_frames`. This does not reduce Crane disk usage or network
traffic. Writing only one PCD per second on Crane itself is not supported by that
unchanged version and cannot be implemented by its capture API.

General does not call the new capture-list route on Crane and does not require
`session.json` complete/success fields absent from the old commit. It waits for
`archive_path` and `size_bytes`, checks download length, the existing manual
session end timestamp, CSV write status, and saved raw PCD entries. A transient
409 while the old Manager is still archiving is retried with the same capture ID.

General stores received archives as
`/var/lib/guardianai/captures/capture_crane_ID.tar.gz`. Its existing individual
status/download routes serve these archives; General's added collection GET
lists its local captures:

```bash
curl http://GENERAL_PC:8765/api/lidar/capture
curl -f http://GENERAL_PC:8765/api/lidar/capture/crane_ID/download -o capture.tar.gz
```

The small `.crane_request.json` in General's existing captures directory stores
an in-flight request ID across restarts. A known ID is kept during disconnections
and download errors. It is deleted only after a successful download or a
confirmed failed job. A lost POST response can leave the ID unknown: the old
Crane API has no listing or idempotency facility, so General pauses remote
requests instead of duplicating a recording. Recover the ID from Crane's existing
Manager output/state into that checkpoint (`manager_url`, `capture_id`,
`request_pending:false`), or clear the checkpoint only after independently
confirming no recording is pending. An unknown ID after Crane Manager restart
also needs operator resolution: its old job table is in-memory. No new remote
recording is silently started in either ambiguous case.

Container originals and Crane host archives remain on Crane. No new automatic
retention/deletion policy is introduced. Failed/incomplete transfers are kept
out of the download-ready list and do not trigger a new recording prematurely.

## General's own existing recorder

General detection recording is configured by this package's own
`config/cx128s2_bev_data_recorder.yaml` and existing
`launch/cx128s2_bev_data_recorder.launch.py`. `run_lidar.sh` and the common
LiDAR bringup have no changes from 8009e8f. Manager does not start a duplicate
recorder or decide vehicle detections.

The package configuration uses the existing `detection_trigger`, observed
TRACKED VEHICLE objects, `recording_duration_sec=10.0` and `pre_buffer_sec=0.0`.
Once this package is running, detection starts a 10-second recording automatically.
Continuous detection does not immediately retrigger; an absent-detection message
after completion rearms the trigger. Its existing steady-clock timer also closes
the event when input messages stop arriving. Manual services remain available.
General Manager only collects closed local sessions using its existing archive
and download workflow, independently of the retained remote Crane request loop.

Use General's existing recorder startup arrangement. To start the package with
its existing launch in the LiDAR container, the working directory must be
`/root/ros2_ws`, so its unchanged relative `output_directory=cx128s2_dataset`
resolves to Manager's expected `/root/ros2_ws/cx128s2_dataset`:

```bash
cd /root/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch cx128s2_bev_data_recorder cx128s2_bev_data_recorder.launch.py
```

Do not launch another instance if General's existing recorder startup already
runs it. A custom storage location must agree with Manager's recorder directory.
The existing `params_file` launch argument can select a custom YAML.

Local General sessions retain `raw/`, `detected/`, `annotations/`, `preview/`,
`frames.csv` and `session.json`. Completion fields added to General's recorder
allow its own Manager to reuse the existing docker cp, compression and download
workflow. These fields are never assumed to exist on Crane.

General's three-hour continuous-vehicle-recognition alert and approximately 60-second
MCAP diagnostic remain local to General. NORMAL request state, MSOP health and
fresh Bridge PointCloud/perception status plus positive `vehicle_count` determine
the interval; no vehicle, stale perception, disconnect or Manager
restart resets it. Once per interval it logs the alert and exposes
`operation_elapsed_sec`, `operation_alert`, `diagnostic_capture` through its
status API. The existing CLI/Docker path records the raw PointCloud, four object
arrays and `/tf`, `/tf_static`, then terminates normally and validates MCAP
metadata before archiving. There is no web frontend in this repository.

## Package settings

The package YAML now defaults to General's 10-second detection policy. Setting
`recording_duration_sec=0` restores the existing variable-duration grace and
post-buffer policy; `pre_buffer_sec` can also be configured independently.
Existing `continuous`, `manual`, `detection_trigger` modes and raw PCD serialization
are reused. No sampling logic is added to the recorder for legacy Crane
compatibility. The separately deployed Crane stays on its old package settings.
