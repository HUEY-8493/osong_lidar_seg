#!/usr/bin/env python3
"""
guardian_manager.py - GuardianAI System Manager

Provides post-boot management:
1. Startup configuration display
2. LiDAR Docker auto-start foundation
3. Docker ROS 2 command execution bridge
4. Optional APC UPS power monitoring with dynamic polling interval
"""

import os
import argparse
import json
import signal
import subprocess
import sys
import threading
import time
import re
import shutil
import shlex
import csv
import io
import urllib.request
import urllib.error
import urllib.parse
import tarfile
from dataclasses import dataclass
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from cx128s2_control import CX128S2Controller, DEFAULT_CACHE_FILE
from mobile_crane_status import (
    CX128S2PacketMonitor,
    LIDAR_MESSAGE_CONTROL_ERROR,
    LIDAR_MESSAGE_RECOVERY_FAILED,
    LIDAR_MESSAGE_UPS_LOW_BATTERY,
    MobileCraneState,
    normalize_working_status,
    GeneralVehicleState,
    RedisPublisher,
)

# ==============================================================================
# CONFIGURATION
# ==============================================================================
# UPS Monitoring
UPS_ONLINE_CHECK_INTERVAL = 0.5  # Polling interval (sec) when STATUS == ONLINE
UPS_ALERT_CHECK_INTERVAL = 5.0    # Polling interval (sec) when STATUS != ONLINE or on error

# UPS TIMELEFT Trigger Settings
# Triggers Docker ROS 2 action when TIMELEFT (minutes) drops to or below threshold
HOST_SHUTDOWN_TIMELEFT_MIN = 3.0   # Docker action / optional host shutdown threshold
UPS_TIMELEFT_ACTION_COMMAND = "ros2 topic list --no-daemon"  # Command to test/verify in Docker
UPS_TIMELEFT_ACTION_DRY_RUN = False  # False = live execution in Docker; True = simulated dry-run

# The selected mode is resolved once at startup. Do not spread mode checks
# through safety code; all policy comes from this immutable profile.
SYSTEM_MODE_DEFAULT = "general"
PERCEPTION_CONFIG_DIR = "/root/ros2_ws/install/cx128s2_bev_perception/share/cx128s2_bev_perception/config"


@dataclass(frozen=True)
class ModeConfig:
    mode: str
    perception_config: str
    ups_monitor_enabled: bool
    lidar_ups_protection_enabled: bool
    host_shutdown_enabled: bool
    bridge_mode: str

    @property
    def rviz_config(self):
        preset = "lidar_crane_work.rviz" if self.mode == "crane" else "lidar.rviz"
        return f"/root/ros2_ws/install/lidar_bringup/share/lidar_bringup/rviz/{preset}"


MODE_CONFIG = {
    "crane": ModeConfig("crane", f"{PERCEPTION_CONFIG_DIR}/cx128s2_bev_perception_crane.yaml",
                        True, True, False, "crane"),
    "general": ModeConfig("general", f"{PERCEPTION_CONFIG_DIR}/cx128s2_bev_perception.yaml",
                          False, False, False, "general"),
}

# LiDAR Docker auto-start
LIDAR_AUTO_START = True
LIDAR_SCRIPT = "/home/guardianai/docker/ros2_jazzy/run_lidar.sh"
ROS2_ENTER_SCRIPT = "/home/guardianai/docker/ros2_jazzy/enter_ros2.sh"
# Stop the detached LiDAR ROS2 bringup before a Manager stop/restart.  The
# container remains running so the next Manager start can launch one clean node.
LIDAR_STOP_ON_MANAGER_STOP = True
LIDAR_BRINGUP_PROCESS = "ros2 launch lidar_bringup lidar_bringup.launch.py"
RVIZ_AUTO_START = True
RVIZ_PROCESS = "rviz2"
LIDAR_RUNTIME_CHECK_INTERVAL_SEC = 10.0

# Docker & ROS 2 environment
USE_SUDO_DOCKER = False           # Set to True if the host requires 'sudo' for docker
DOCKER_CONTAINER_NAME = "ros2_jazzy_lidar"
ROS_SETUP_SCRIPT = "/opt/ros/jazzy/setup.bash"

# System Shutdown & Host Command Settings
SHUTDOWN_ENABLED = False           # Set True to enable host shutdown when TIMELEFT is low
SHUTDOWN_COMMAND = "sudo shutdown -h now"  # Sudo command to turn off host (or any custom command)
SHUTDOWN_DRY_RUN = False           # Safety toggle: True = simulate shutdown without turning off host
SHUTDOWN_DELAY_SEC = 5.0          # Delay (seconds) after Docker ROS 2 action before shutdown

# CX128S2 host-side control.  None selects the route to CX128S2 automatically.
CX128S2_ENABLED = True
CX128S2_IP = "192.168.1.200"
# Dedicated CX128S2 NIC. Set to None only on deployments where route discovery
# is known to select the LiDAR interface correctly.
CX128S2_INTERFACE = "enp2s0"
CX128S2_CACHE_FILE = str(DEFAULT_CACHE_FILE)
# GUARDIAN_API_BIND = "127.0.0.1"   # localhost only (default)
GUARDIAN_API_BIND = "0.0.0.0"
GUARDIAN_API_PORT = 8765
GUARDIAN_BRIDGE_TOKEN = os.environ.get("GUARDIAN_BRIDGE_TOKEN", "")
BRIDGE_HEALTH_TIMEOUT_SEC = 2.0

CAPTURE_ENABLED = True
# General background recording is opt-in; explicit capture API requests remain available.
GENERAL_AUTO_CAPTURE_ENABLED = False
CAPTURE_MIN_DURATION_SEC = 1
CAPTURE_MAX_DURATION_SEC = 120
CAPTURE_ARCHIVE_DIR = Path("/var/lib/guardianai/captures")
CAPTURE_MIN_FREE_BYTES = 1 * 1024 * 1024 * 1024
CAPTURE_RECORDER_DIR = Path("/root/ros2_ws/cx128s2_dataset")
GENERAL_ALERT_AFTER_SEC = 3 * 60 * 60
GENERAL_ROSBAG_DURATION_SEC = 60
CRANE_MANAGER_URL = os.environ.get("GUARDIAN_CRANE_MANAGER_URL", "http://192.168.223.151:8765").rstrip("/")
CRANE_CAPTURE_DURATION_SEC = 60
RECORDER_START_SERVICE = "/start_recording"
RECORDER_STOP_SERVICE = "/stop_recording"

# External status contract. Redis failures never gate LiDAR or UPS safety
# actions, and the schema is selected by the crane/general runtime mode.
CX128S2_MSOP_PORT = 2368
CX128S2_PACKET_TIMEOUT_SEC = 2.0
# The configured Redis stream receives the selected runtime-mode payload.
REDIS_ENABLED = True
REDIS_HOST = "192.168.223.20"
REDIS_PORT = 6379
REDIS_KEY = "SP-OS-RW-LIDAR01"
REDIS_CRANE_KEY = "SP-OS-RW-LIDAR02"
REDIS_PUBLISH_INTERVAL_SEC = 1.0

# ==============================================================================
# GLOBAL STATE & SIGNALS
# ==============================================================================
_running = True
_timeleft_action_triggered = False
_api_server = None
_lidar_controller = None
_crane_state = MobileCraneState()
_crane_state_from_bridge = False
_vehicle_state = GeneralVehicleState()
_packet_monitor = None
_redis_publisher = None
_runtime_mode_config = MODE_CONFIG[SYSTEM_MODE_DEFAULT]
_last_ups = {"enabled": False}


def _external_state():
    """Return the Redis state model selected by the immutable runtime profile."""
    return _vehicle_state if _runtime_mode_config.mode == "general" else _crane_state


class BridgeState:
    """Thread-safe, schema-limited state supplied by Docker-local ROS bridge."""
    def __init__(self):
        self._lock = threading.Lock()
        self.payload = None
        self.received_at = None

    def update(self, payload, expected_mode):
        if not isinstance(payload, dict) or payload.get("mode") != expected_mode:
            raise ValueError("bridge payload mode does not match manager mode")
        if not isinstance(payload.get("lidar"), dict) or not isinstance(payload.get("perception"), dict):
            raise ValueError("bridge payload requires lidar and perception objects")
        with self._lock:
            if (expected_mode == "crane" and self.payload and
                    not payload["lidar"].get("perception_active", False)):
                # Health-only/reconnecting bridge messages cannot erase the last
                # sensor observation. Freshness stays in the separate health flags.
                payload = {**payload, "perception": self.payload["perception"]}
            self.payload, self.received_at = payload, time.monotonic()

    def snapshot(self, timeout_sec):
        with self._lock:
            payload, received_at = self.payload, self.received_at
        fresh = received_at is not None and time.monotonic() - received_at <= timeout_sec
        if fresh and payload.get("mode") == "general":
            health = payload["lidar"]
            if not (health.get("pointcloud_active", False) and health.get("perception_active", False)):
                # Mask unavailable measurements at the public boundary only.
                # Retain the received state and all perception-side identity memory.
                payload = {**payload, "perception": {
                    **payload["perception"], "object_detected": False,
                    "object_count": 0, "tracked_object_count": 0, "moving_object_count": 0,
                    "vehicle_count": 0, "vehicle_exist": 0, "vehicle_status": "",
                    "vehicle_distance": 0.0, "vehicle_length": 0.0,
                }}
        retain_crane = payload is not None and payload.get("mode") == "crane"
        return (payload if fresh or retain_crane else None), fresh


_bridge_state = BridgeState()


class CaptureManager:
    """One bounded recorder job at a time; API inputs never become shell text."""
    def __init__(self):
        self._lock = threading.Lock()
        self._active_id = None
        self._jobs = {}
        self._archive_lock = threading.Lock()
        self._collecting = False
        self._rosbag_active_id = None
        self._general_started_at = None
        self._general_alert = None
        self._crane_busy = False
        self._crane_capture_id = None
        self._crane_error = ""
        self._crane_request_pending = False

    def _ros_service(self, service):
        command = (
            f"source {ROS_SETUP_SCRIPT} && source /root/ros2_ws/install/setup.bash && "
            f"timeout 30s ros2 service call {service} std_srvs/srv/Trigger '{{}}'"
        )
        result = _docker_exec(command, timeout=35.0)
        success = re.search(r"\bsuccess\s*[:=]\s*(true|false)\b", result.stdout, re.IGNORECASE)
        if result.returncode == 0 and (not success or success.group(1).lower() != "true"):
            result.returncode = 1
            result.stderr = result.stdout.strip() or "recorder did not confirm success"
        return result

    @staticmethod
    def _recorder_path(output):
        """Accept both ROS CLI YAML and Python repr response formats."""
        patterns = (
            r"message:\s*['\"]?([^'\"\n]+)",
            r"message\s*=\s*['\"]([^'\"]+)['\"]",
        )
        for pattern in patterns:
            match = re.search(pattern, output)
            if match and match.group(1).strip():
                return match.group(1).strip()
        return ""

    def start(self, duration_sec):
        if not CAPTURE_ENABLED:
            return {"success": False, "error": "capture is disabled"}
        if not isinstance(duration_sec, (int, float)) or isinstance(duration_sec, bool):
            return {"success": False, "error": "duration_sec must be numeric"}
        duration_sec = int(duration_sec)
        if not CAPTURE_MIN_DURATION_SEC <= duration_sec <= CAPTURE_MAX_DURATION_SEC:
            return {"success": False, "error": f"duration_sec must be {CAPTURE_MIN_DURATION_SEC}..{CAPTURE_MAX_DURATION_SEC}"}
        try:
            CAPTURE_ARCHIVE_DIR.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(CAPTURE_ARCHIVE_DIR).free < CAPTURE_MIN_FREE_BYTES:
                return {"success": False, "error": "insufficient free space for capture"}
        except OSError as exc:
            return {"success": False, "error": str(exc)}
        with self._lock:
            if self._active_id:
                return {"success": False, "error": "a capture is already recording"}
            try:
                result = self._ros_service(RECORDER_START_SERVICE)
            except Exception as exc:
                return {"success": False, "error": f"recorder start invocation failed: {exc}"}
            if result.returncode != 0:
                return {"success": False, "error": result.stderr.strip() or "recorder start failed"}
            capture_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            recorder_path = self._recorder_path(result.stdout)
            if recorder_path and not Path(recorder_path).is_absolute():
                recorder_path = str(Path("/root/ros2_ws") / recorder_path)
            self._active_id = capture_id
            self._jobs[capture_id] = {"capture_id": capture_id, "state": "recording", "duration_sec": duration_sec,
                                      "started_at": time.time(), "frame_count": None, "size_bytes": None,
                                      "recorder_path": recorder_path}
            threading.Thread(target=self._finish, args=(capture_id,), daemon=True, name=f"capture-{capture_id}").start()
            return {"success": True, "capture_id": capture_id, "state": "recording", "duration_sec": duration_sec}

    def _finish(self, capture_id):
        with self._lock:
            job = self._jobs[capture_id]
        try:
            if job.get("kind") == "rosbag":
                # timeout sends SIGTERM to the container-side CLI, which closes the bag.
                topics = ["/ch/lslidar_point_cloud", "/detected_objects", "/tracked_objects",
                          "/object_motion_states", "/road_vehicle_tracks", "/tf", "/tf_static"]
                disk_check = "import shutil; assert shutil.disk_usage(" + repr(str(CAPTURE_RECORDER_DIR)) + ").free >= " + str(CAPTURE_MIN_FREE_BYTES) + ", 'insufficient free space for rosbag'"
                command = (f"source {ROS_SETUP_SCRIPT} && source /root/ros2_ws/install/setup.bash && "
                           f"mkdir -p {shlex.quote(str(CAPTURE_RECORDER_DIR))} && "
                           f"python3 -c {shlex.quote(disk_check)} && "
                           f"timeout --signal=TERM --kill-after=15s {GENERAL_ROSBAG_DURATION_SEC}s "
                           "ros2 bag record --disable-keyboard-controls -s mcap -o "
                           + shlex.quote(job["recorder_path"]) + " --topics " + " ".join(topics))
                result = _docker_exec(command, timeout=GENERAL_ROSBAG_DURATION_SEC + 30)
                if result.returncode not in (0, 124):
                    raise RuntimeError(result.stderr.strip() or "rosbag recording failed")
            elif not job.get("automatic"):
                time.sleep(job["duration_sec"])
                result = self._ros_service(RECORDER_STOP_SERVICE)
                if result.returncode != 0:
                    raise RuntimeError(result.stderr.strip() or "recorder stop failed")
                job["recorder_path"] = self._recorder_path(result.stdout)

            recorder_path = Path(job.get("recorder_path") or "")
            if not recorder_path.is_absolute():
                recorder_path = Path("/root/ros2_ws") / recorder_path
            if recorder_path.parent != CAPTURE_RECORDER_DIR or ".." in recorder_path.parts:
                raise RuntimeError("recorder returned an unexpected output path")
            job["recorder_path"] = str(recorder_path)
            job["state"] = "archiving"
            with self._archive_lock:
                CAPTURE_ARCHIVE_DIR.mkdir(parents=True, exist_ok=True)
                archive = CAPTURE_ARCHIVE_DIR / f"capture_{capture_id}.tar.gz"
                if not archive.is_file():
                    if shutil.disk_usage(CAPTURE_ARCHIVE_DIR).free < CAPTURE_MIN_FREE_BYTES:
                        raise RuntimeError("insufficient free space for capture archive")
                    staging = CAPTURE_ARCHIVE_DIR / capture_id
                    # An interrupted prior copy must not cause docker cp to nest directories.
                    if staging.exists():
                        shutil.rmtree(staging)
                    docker_cmd = ["sudo", "docker"] if USE_SUDO_DOCKER else ["docker"]
                    subprocess.run(docker_cmd + ["cp", f"{DOCKER_CONTAINER_NAME}:{recorder_path}", str(staging)],
                                   capture_output=True, text=True, timeout=60.0, check=True)
                    if job.get("kind") == "rosbag":
                        metadata = staging / "metadata.yaml"
                        if not metadata.is_file() or not any(p.stat().st_size for p in staging.glob("*.mcap")):
                            raise RuntimeError("rosbag did not produce a closed MCAP bag with metadata")
                        messages = re.search(r"\bmessage_count:\s*(\d+)", metadata.read_text())
                        if not messages or int(messages.group(1)) == 0:
                            raise RuntimeError("rosbag did not produce a closed MCAP bag with metadata")
                    else:
                        session = json.loads((staging / "session.json").read_text())
                        if session.get("complete") is not True or session.get("success") is not True:
                            raise RuntimeError("recorder did not confirm successful file completion")
                    temporary = archive.with_suffix(archive.suffix + ".tmp")
                    with tarfile.open(temporary, "w:gz") as output:
                        output.add(staging, arcname=".")
                    temporary.replace(archive)
                    job["frame_count"] = len(list((staging / "raw").glob("*.pcd")))
                    shutil.rmtree(staging)
                with self._lock:
                    job.update(state="complete", error="", completed_at=time.time(),
                               archive_path=str(archive), size_bytes=archive.stat().st_size)
        except Exception as exc:
            with self._lock:
                job.update(state="failed", error=str(exc), completed_at=time.time())
            log("CAPTURE ERROR", f"{capture_id}: {exc}")
        finally:
            with self._lock:
                if self._active_id == capture_id:
                    self._active_id = None
                if self._rosbag_active_id == capture_id:
                    self._rosbag_active_id = None

    def collect_completed(self):
        """Import closed automatic sessions into the existing copy/download workflow."""
        with self._lock:
            if self._collecting:
                return
            self._collecting = True
        try:
            script = ("import json,pathlib; paths=[]\n"
                      f"for p in pathlib.Path({str(CAPTURE_RECORDER_DIR)!r}).glob('recording_*/session.json'):\n"
                      " try:\n"
                      "  if json.loads(p.read_text()).get('complete') is True: paths.append(str(p.parent))\n"
                      " except (OSError,ValueError): pass\n"
                      "print(json.dumps(paths))")
            result = _docker_exec("python3 -c " + shlex.quote(script), timeout=10.0)
            if result.returncode != 0:
                return
            for recorder_path in json.loads(result.stdout):
                path = Path(recorder_path)
                if path.parent != CAPTURE_RECORDER_DIR or not re.fullmatch(r"recording_[A-Za-z0-9_]+", path.name):
                    continue
                capture_id = "auto_" + path.name
                with self._lock:
                    if capture_id in self._jobs or any(j.get("recorder_path") == recorder_path for j in self._jobs.values()):
                        continue
                    self._jobs[capture_id] = {"capture_id": capture_id, "state": "archiving", "automatic": True,
                                              "recorder_path": recorder_path, "frame_count": None, "size_bytes": None}
                self._finish(capture_id)
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            log("CAPTURE COLLECTION ERROR", str(exc))
        finally:
            with self._lock:
                self._collecting = False

    def capture_crane(self):
        """General requests, polls and downloads Crane's existing HTTP capture API."""
        if not CRANE_MANAGER_URL or _runtime_mode_config.mode != "general":
            return
        with self._lock:
            if self._crane_busy:
                return
            self._crane_busy = True
        try:
            # Verify the endpoint's role; never accidentally command General itself.
            with urllib.request.urlopen(CRANE_MANAGER_URL + "/api/lidar/status", timeout=10) as response:
                status = json.load(response)
            if status.get("system_mode") != "crane":
                raise RuntimeError("configured Crane Manager is not in crane mode")
            CAPTURE_ARCHIVE_DIR.mkdir(parents=True, exist_ok=True)
            checkpoint = CAPTURE_ARCHIVE_DIR / ".crane_request.json"
            if self._crane_capture_id is None and checkpoint.is_file():
                saved = json.loads(checkpoint.read_text())
                if saved.get("manager_url") != CRANE_MANAGER_URL:
                    raise RuntimeError("pending Crane request belongs to another Manager URL")
                self._crane_capture_id = saved.get("capture_id")
                self._crane_request_pending = saved.get("request_pending", False)
            if self._crane_capture_id is None:
                if self._crane_request_pending:
                    raise RuntimeError("Crane POST outcome is unknown; recover its capture_id in .crane_request.json before retrying")
                if not status.get("bridge_connected") or not status.get("lidar", {}).get("pointcloud_active"):
                    raise RuntimeError("Crane PointCloud is not active; capture postponed")
                if shutil.disk_usage(CAPTURE_ARCHIVE_DIR).free < CAPTURE_MIN_FREE_BYTES:
                    raise RuntimeError("insufficient free space for Crane download")
                # The legacy API has no list or idempotency endpoint. Save intent
                # before POST; an ambiguous lost response must not start another job.
                self._crane_request_pending = True
                checkpoint_tmp = checkpoint.with_suffix(".tmp")
                checkpoint_tmp.write_text(json.dumps({"manager_url": CRANE_MANAGER_URL, "request_pending": True}))
                checkpoint_tmp.replace(checkpoint)
                request = urllib.request.Request(CRANE_MANAGER_URL + "/api/lidar/capture",
                                                 data=json.dumps({"duration_sec": CRANE_CAPTURE_DURATION_SEC}).encode(),
                                                 headers={"Content-Type": "application/json"}, method="POST")
                try:
                    with urllib.request.urlopen(request, timeout=40) as response:
                        started = json.load(response)
                except urllib.error.HTTPError as exc:
                    if 400 <= exc.code < 500:
                        self._crane_request_pending = False
                        checkpoint.unlink(missing_ok=True)
                    raise
                except urllib.error.URLError as exc:
                    if isinstance(exc.reason, OSError) and exc.reason.errno in (111, 101, 113):
                        self._crane_request_pending = False
                        checkpoint.unlink(missing_ok=True)
                    raise
                if started.get("success") is not True:
                    self._crane_request_pending = False
                    checkpoint.unlink(missing_ok=True)
                    raise RuntimeError(started.get("error", "Crane did not confirm capture start"))
                remote_id = str(started.get("capture_id", ""))
                if not re.fullmatch(r"[A-Za-z0-9_]+", remote_id):
                    raise RuntimeError("Crane returned an invalid capture ID")
                self._crane_capture_id = remote_id
                checkpoint_tmp.write_text(json.dumps({"manager_url": CRANE_MANAGER_URL, "capture_id": remote_id,
                                                       "request_pending": False}))
                checkpoint_tmp.replace(checkpoint)
                self._crane_request_pending = False
            remote_id = self._crane_capture_id
            if not isinstance(remote_id, str) or not re.fullmatch(r"[A-Za-z0-9_]+", remote_id):
                raise RuntimeError("pending Crane capture ID is invalid")
            endpoint = CRANE_MANAGER_URL + "/api/lidar/capture/" + urllib.parse.quote(remote_id, safe="")
            with urllib.request.urlopen(endpoint, timeout=10) as response:
                job = json.load(response)
            if job.get("state") == "failed":
                self._crane_capture_id = None
                checkpoint.unlink(missing_ok=True)
                raise RuntimeError(job.get("error", "Crane capture failed"))
            # 8009e8f marks complete before copying/archiving. Wait for the
            # actual archive fields (and still accept a transient download 409).
            if job.get("state") == "complete" and job.get("archive_path") and job.get("size_bytes", 0):
                CAPTURE_ARCHIVE_DIR.mkdir(parents=True, exist_ok=True)
                if shutil.disk_usage(CAPTURE_ARCHIVE_DIR).free < CAPTURE_MIN_FREE_BYTES:
                    raise RuntimeError("insufficient free space for Crane download")
                local_id = "crane_" + remote_id
                archive = CAPTURE_ARCHIVE_DIR / f"capture_{local_id}.tar.gz"
                temporary = archive.with_suffix(archive.suffix + ".tmp")
                with self._archive_lock:
                    with urllib.request.urlopen(endpoint + "/download", timeout=60) as response, temporary.open("wb") as output:
                        expected_size = int(response.headers["Content-Length"])
                        shutil.copyfileobj(response, output)
                    if temporary.stat().st_size != expected_size or (job.get("size_bytes") is not None and expected_size != job["size_bytes"]):
                        raise RuntimeError("Crane archive download was incomplete")
                    # Legacy session.json has no complete/success flags. Validate
                    # its existing end stamp and CSV/PCD output, then select one raw
                    # frame per source second on General without changing Crane.
                    selected_archive = archive.with_suffix(archive.suffix + ".selected.tmp")
                    with tarfile.open(temporary, "r:gz") as downloaded:
                        members = {member.name.removeprefix("./"): member for member in downloaded.getmembers()}
                        if any(name.startswith("/") or ".." in Path(name).parts for name in members):
                            raise RuntimeError("Crane archive contains an invalid path")
                        for name in ("session.json", "frames.csv"):
                            if name not in members or not members[name].isfile():
                                raise RuntimeError("Crane archive lacks " + name)
                        session = json.load(downloaded.extractfile(members["session.json"]))
                        if session.get("reason") != "manual" or session.get("event_end_stamp_ns", 0) <= 0:
                            raise RuntimeError("legacy Crane session did not finish a manual recording")
                        lines = downloaded.extractfile(members["frames.csv"]).read().decode().splitlines()
                        if not lines:
                            raise RuntimeError("Crane frame index is empty")
                        header = next(csv.reader([lines[0]]))
                        rows = []
                        selected = []
                        buckets = set()
                        for line in lines[1:]:
                            # 8009e8f leaves the final empty quoted field unclosed.
                            values = next(csv.reader([line + ('"' if line.endswith(',"') else '')]))
                            if len(values) != len(header):
                                raise RuntimeError("Crane frame index is malformed")
                            row = dict(zip(header, values))
                            raw_path = row["raw_pcd_path"]
                            if "WRITE_FAILED" in row["save_status"] or "DISK_LOW" in row["save_status"]:
                                raise RuntimeError("Crane reported a frame write failure")
                            if not raw_path or raw_path not in members or not members[raw_path].isfile() or not members[raw_path].size:
                                raise RuntimeError("Crane frame has no saved raw PCD")
                            rows.append(row)
                            bucket = int(row["stamp_sec"])
                            if bucket not in buckets:
                                buckets.add(bucket)
                                selected.append(row)
                        if not selected:
                            raise RuntimeError("Crane archive contains no raw PCD frames")
                        kept = {row["raw_pcd_path"] for row in selected}
                        kept.update(row["annotation_path"] for row in selected if row["annotation_path"])
                        detected = [row["detected_pcd_path"] + "/" for row in selected if row["detected_pcd_path"]]
                        csv_output = io.StringIO()
                        writer = csv.DictWriter(csv_output, fieldnames=header)
                        writer.writeheader()
                        writer.writerows(selected)
                        session.update(general_save_interval_sec=1.0, general_original_raw_frames=len(rows),
                                       general_retained_raw_frames=len(selected))
                        with tarfile.open(selected_archive, "w:gz") as output:
                            for name, member in members.items():
                                if name in ("frames.csv", "session.json"):
                                    payload = (csv_output.getvalue().encode() if name == "frames.csv" else
                                               json.dumps(session).encode())
                                    info = tarfile.TarInfo("./" + name)
                                    info.size = len(payload)
                                    output.addfile(info, io.BytesIO(payload))
                                elif member.isfile() and (name in kept or name.startswith("preview/") or
                                                         any(name.startswith(prefix) for prefix in detected)):
                                    output.addfile(member, downloaded.extractfile(member))
                        selected_archive.replace(archive)
                    temporary.unlink(missing_ok=True)
                with self._lock:
                    self._jobs[local_id] = {"capture_id": local_id, "state": "complete", "kind": "crane",
                                            "remote_capture_id": remote_id, "archive_path": str(archive),
                                            "size_bytes": archive.stat().st_size, "frame_count": len(selected),
                                            "source_frame_count": len(rows),
                                            "completed_at": time.time()}
                self._crane_capture_id = None
                checkpoint.unlink(missing_ok=True)
                log("CRANE CAPTURE", f"Downloaded {remote_id}; next poll will start the next capture")
            self._crane_error = ""
        except Exception as exc:
            self._crane_error = str(exc)
            log("CRANE CAPTURE ERROR", str(exc))
        finally:
            with self._lock:
                self._crane_busy = False

    def start_rosbag(self):
        """Run one diagnostic bag and reuse _finish for archiving and download."""
        with self._lock:
            if self._rosbag_active_id:
                return self._rosbag_active_id
            capture_id = "bag_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self._rosbag_active_id = capture_id
            self._jobs[capture_id] = {"capture_id": capture_id, "state": "recording", "kind": "rosbag",
                                      "duration_sec": GENERAL_ROSBAG_DURATION_SEC, "started_at": time.time(),
                                      "recorder_path": str(CAPTURE_RECORDER_DIR / capture_id),
                                      "frame_count": None, "size_bytes": None}
            threading.Thread(target=self._finish, args=(capture_id,), daemon=True, name=capture_id).start()
            return capture_id

    def get(self, capture_id=None):
        with self._lock:
            # Completed archives survive a Manager restart; no separate index/database.
            if CAPTURE_ARCHIVE_DIR.is_dir():
                for archive in CAPTURE_ARCHIVE_DIR.glob("capture_*.tar.gz"):
                    stored_id = archive.name[len("capture_"):-len(".tar.gz")]
                    if stored_id not in self._jobs:
                        self._jobs[stored_id] = {"capture_id": stored_id, "state": "complete",
                                                 "archive_path": str(archive), "size_bytes": archive.stat().st_size}
            if capture_id is None:
                return [dict(job) for job in self._jobs.values()]
            job = self._jobs.get(capture_id)
            return dict(job) if job else None


_capture_manager = CaptureManager()


def build_status_payload(controller):
    """Keep existing controller fields and add independent health layers."""
    result = controller.status()
    operation_started_at = _capture_manager._general_started_at
    operation_alert = _capture_manager._general_alert
    bridge, bridge_fresh = _bridge_state.snapshot(BRIDGE_HEALTH_TIMEOUT_SEC)
    packet_healthy, _ = _packet_monitor.health() if _packet_monitor else (False, "monitor_unavailable")
    runtime = get_lidar_runtime_status()
    bridge_lidar = bridge.get("lidar", {}) if bridge else {}
    result.update({
        "system_mode": _runtime_mode_config.mode,
        "lidar": {
            "status": _external_state().build_payload()["lidar_status"],
            "packet_active": packet_healthy,
            **runtime,
            "pointcloud_active": bool(bridge_lidar.get("pointcloud_active", False)) if bridge_fresh else False,
            "last_pointcloud_age_ms": bridge_lidar.get("last_pointcloud_age_ms") if bridge_fresh else None,
            "perception_active": bool(bridge_lidar.get("perception_active", False)) if bridge_fresh else False,
            "last_perception_age_ms": bridge_lidar.get("last_perception_age_ms") if bridge_fresh else None,
        },
        "perception": (bridge.get("perception", {}) if bridge and
                       (bridge_fresh or _runtime_mode_config.mode == "crane") else {}),
        "bridge_connected": bridge_fresh,
        "ups": dict(_last_ups),
        "operation_elapsed_sec": (max(0.0, time.monotonic() - operation_started_at)
                                  if operation_started_at is not None else 0.0),
        "crane_capture": {"manager_url": CRANE_MANAGER_URL, "capture_id": _capture_manager._crane_capture_id,
                          "busy": _capture_manager._crane_busy, "error": _capture_manager._crane_error,
                          "request_pending": _capture_manager._crane_request_pending}
                         if _runtime_mode_config.mode == "general" else None,
        "operation_alert": dict(operation_alert) if operation_alert else None,
        "diagnostic_capture": (_capture_manager.get(operation_alert["capture_id"])
                               if operation_alert else None),
    })
    if _runtime_mode_config.mode == "general" and not bridge_fresh:
        result["perception"] = {"vehicle_exist": 0, "vehicle_status": "", "vehicle_count": 0,
                                "object_detected": False, "object_count": 0,
                                "tracked_object_count": 0, "moving_object_count": 0,
                                "vehicle_distance": 0.0, "vehicle_length": 0.0}
    return result


def _json_handler(controller):
    class GuardianAPIHandler(BaseHTTPRequestHandler):
        def _respond(self, status, body):
            encoded = json.dumps(body, sort_keys=True).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

        def do_GET(self):
            if self.path == "/api/lidar/status":
                result = build_status_payload(controller)
                result["redis_payload"] = build_external_payload()
                result["ros2_runtime"] = get_lidar_runtime_status()
                self._respond(200, result)
            elif self.path == "/api/lidar/capture":
                self._respond(200, {"captures": _capture_manager.get()})
            elif self.path.startswith("/api/lidar/capture/"):
                parts = self.path.split("/")
                capture_id = parts[4] if len(parts) > 4 else ""
                job = _capture_manager.get(capture_id)
                if not job:
                    self._respond(404, {"success": False, "error": "capture not found"})
                elif len(parts) == 6 and parts[5] == "download":
                    archive = Path(job.get("archive_path", ""))
                    if job["state"] != "complete" or not archive.is_file() or archive.parent != CAPTURE_ARCHIVE_DIR:
                        self._respond(409, {"success": False, "error": "capture archive is not available"})
                    else:
                        self.send_response(200)
                        self.send_header("Content-Type", "application/gzip")
                        self.send_header("Content-Length", str(archive.stat().st_size))
                        self.send_header("Content-Disposition", f'attachment; filename="{archive.name}"')
                        self.end_headers()
                        with archive.open("rb") as source:
                            shutil.copyfileobj(source, self.wfile)
                else:
                    self._respond(200, job)
            else:
                self._respond(404, {"success": False, "error": "Not found"})

        def do_POST(self):
            global _crane_state_from_bridge
            if self.path == "/api/lidar/standby":
                result = command_lidar("standby")
            elif self.path == "/api/lidar/normal":
                result = command_lidar("normal")
            elif self.path == "/api/crane/state":
                try:
                    if _runtime_mode_config.mode != "crane":
                        raise ValueError("/api/crane/state is available only in crane mode")
                    content_length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(content_length))
                    update_crane_state(body["working_status"], body["object_length"], body["crane_moving"],
                                       body.get("sensor_timestamp_ms"))
                    self._respond(200, {"success": True, "redis_payload": build_external_payload()})
                except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
                    self._respond(400, {"success": False, "error": str(exc)})
                return
            elif self.path == "/api/internal/bridge":
                if GUARDIAN_BRIDGE_TOKEN and self.headers.get("X-Guardian-Bridge-Token") != GUARDIAN_BRIDGE_TOKEN:
                    self._respond(403, {"success": False, "error": "invalid bridge token"})
                    return
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))))
                    if (_runtime_mode_config.mode == "crane" and isinstance(body, dict) and
                            isinstance(body.get("lidar"), dict) and body["lidar"].get("perception_active", False)):
                        normalize_working_status(body["perception"]["working_status"])
                    _bridge_state.update(body, _runtime_mode_config.bridge_mode)
                    # The bridge is authoritative for automatic external state.
                    # The legacy crane endpoint remains available in crane mode.
                    if _runtime_mode_config.mode == "crane":
                        perception = body["perception"]
                        if body["lidar"].get("perception_active", False):
                            _crane_state.update_crane_state(
                                perception["working_status"], float(perception["beam_length"]),
                                int(perception["crane_moving"]), int(perception["sensor_timestamp_ms"]),
                            )
                        # Without a new perception frame retain the last work,
                        # length, motion and original sensor timestamp.
                        _crane_state_from_bridge = True
                    else:
                        perception = body["perception"]
                        _vehicle_state.update_vehicle_state(
                            int(perception["vehicle_exist"]), str(perception["vehicle_status"]),
                            float(perception["vehicle_distance"]), int(perception["sensor_timestamp_ms"]),
                            float(perception["vehicle_length"]),
                        )
                    self._respond(200, {"success": True})
                except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
                    self._respond(400, {"success": False, "error": str(exc)})
                return
            elif self.path == "/api/lidar/capture":
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))))
                    result = _capture_manager.start(body.get("duration_sec", 30))
                    self._respond(202 if result.get("success") else 400, result)
                except (ValueError, TypeError, json.JSONDecodeError, OSError, subprocess.SubprocessError) as exc:
                    self._respond(400, {"success": False, "error": str(exc)})
                except Exception as exc:
                    log("CAPTURE API ERROR", repr(exc))
                    self._respond(500, {"success": False, "error": "capture request failed; check manager logs"})
                return
            else:
                self._respond(404, {"success": False, "error": "Not found"})
                return
            self._respond(200 if result["success"] else 503, result)

        def log_message(self, fmt, *args):
            log("LIDAR API", fmt % args)
    return GuardianAPIHandler


def start_lidar_api(controller):
    global _api_server
    try:
        _api_server = ThreadingHTTPServer((GUARDIAN_API_BIND, GUARDIAN_API_PORT), _json_handler(controller))
        thread = threading.Thread(target=_api_server.serve_forever, name="guardian-lidar-api", daemon=True)
        thread.start()
        log("LIDAR API", f"Listening on http://{GUARDIAN_API_BIND}:{GUARDIAN_API_PORT}")
        return True
    except OSError as exc:
        log("LIDAR API ERROR", f"Could not start API: {exc}")
        return False


def stop_lidar_api():
    global _api_server
    if _api_server is not None:
        _api_server.shutdown()
        _api_server.server_close()
        _api_server = None


def update_crane_state(working_status: str | int, object_length: float, crane_moving: int,
                       sensor_timestamp_ms: int | None = None):
    """Integration point for the future perception/status module."""
    global _crane_state_from_bridge
    _crane_state.update_crane_state(working_status, object_length, crane_moving, sensor_timestamp_ms)
    _crane_state_from_bridge = False


def build_external_payload() -> dict:
    """Return the six-key Redis schema selected by --mode."""
    payload = _external_state().build_payload()
    if _runtime_mode_config.mode == "general":
        bridge, fresh = _bridge_state.snapshot(BRIDGE_HEALTH_TIMEOUT_SEC)
        if not fresh or not bridge["perception"].get("vehicle_exist", 0):
            payload.update(vehicle_exist=0, vehicle_status="", vehicle_distance=0.0, vehicle_length=0.0)
    # Crane retains its last observation through input/bridge loss. Health is
    # updated independently; timestamp remains the original sensor frame time.
    return payload


def command_lidar(mode: str, standby_reason=None) -> dict:
    """Issue a serialized command and atomically update the public LiDAR state."""
    if _lidar_controller is None:
        result = {"success": False, "error": "CX128S2 controller is unavailable"}
    else:
        result = _lidar_controller.standby() if mode == "standby" else _lidar_controller.normal()
    if result.get("success"):
        _external_state().request_lidar_state(mode, standby_reason)
    else:
        _external_state().set_lidar_state(0, LIDAR_MESSAGE_CONTROL_ERROR)
    return result


def refresh_lidar_health():
    """Map packet health plus requested state onto the public LiDAR fields."""
    if _packet_monitor is None:
        return
    requested_state, normal_requested_at = _external_state().lidar_context()
    if requested_state == "standby":
        return  # command_lidar already selected standby or ups_low_battery
    healthy, message = _packet_monitor.health()
    if healthy:
        _external_state().set_lidar_state(1, "")
    elif normal_requested_at and time.monotonic() - normal_requested_at > CX128S2_PACKET_TIMEOUT_SEC:
        _external_state().set_lidar_state(0, LIDAR_MESSAGE_RECOVERY_FAILED)
    else:
        _external_state().set_lidar_state(0, message)


def handle_shutdown_signal(signum, frame):
    """Handles SIGINT and SIGTERM for clean shutdown."""
    global _running
    sig_name = signal.Signals(signum).name
    log("MANAGER", f"Shutdown requested via {sig_name}. Exiting gracefully...")
    _running = False
    stop_lidar_ros2_bringup()
    stop_lidar_api()


# ==============================================================================
# LOGGING HELPER
# ==============================================================================
def log(tag: str, message: str):
    """Prints a formatted timestamped log message."""
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] [{tag}] {message}", flush=True)


# ==============================================================================
# 1. LIDAR DOCKER AUTO-START
# ==============================================================================
def _docker_exec(command, timeout=5.0):
    docker_cmd = ["sudo", "docker"] if USE_SUDO_DOCKER else ["docker"]
    return subprocess.run(
        docker_cmd + ["exec", DOCKER_CONTAINER_NAME, "bash", "-lc", command],
        capture_output=True, text=True, timeout=timeout, check=False,
    )


def _container_running():
    docker_cmd = ["sudo", "docker"] if USE_SUDO_DOCKER else ["docker"]
    try:
        result = subprocess.run(
            docker_cmd + ["ps", "-q", "-f", f"name=^{DOCKER_CONTAINER_NAME}$"],
            capture_output=True, text=True, timeout=5.0, check=False,
        )
        return result.returncode == 0 and bool(result.stdout.strip())
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return False


def _ros_process_running(process):
    if not _container_running():
        return False
    pattern = process.replace(process.split()[0], f"[{process[0]}]{process[1:len(process.split()[0])]}", 1)
    try:
        return _docker_exec(f"pgrep -f '{pattern}' >/dev/null").returncode == 0
    except Exception:
        return False


def get_lidar_runtime_status():
    """Docker/ROS process status, separate from actual MSOP health."""
    return {
        "container_running": _container_running(),
        "lidar_bringup_running": _ros_process_running(LIDAR_BRINGUP_PROCESS),
        "rviz2_running": _ros_process_running(RVIZ_PROCESS),
    }


def start_lidar_docker(dry_run: bool = False) -> bool:
    """
    Executes the LiDAR Docker launch script if enabled.
    If the script is not found, automatically falls back to checking/starting
    the Docker container (DOCKER_CONTAINER_NAME) directly.
    """
    if not LIDAR_AUTO_START:
        log("LIDAR", "Auto-start is disabled (LIDAR_AUTO_START=False).")
        return True

    if _ros_process_running(LIDAR_BRINGUP_PROCESS):
        log("LIDAR", "LiDAR ROS2 bringup is already running.")
        return True

    # 1. If custom script exists, execute it
    if os.path.isfile(LIDAR_SCRIPT):
        log("LIDAR", f"Found startup script: {LIDAR_SCRIPT}")
        if dry_run:
            log("LIDAR DRY-RUN", f"Would execute: {LIDAR_SCRIPT}")
            return True
        try:
            log("LIDAR", f"Executing {LIDAR_SCRIPT}...")
            child_env = os.environ.copy()
            child_env.update({
                "GUARDIAN_SYSTEM_MODE": _runtime_mode_config.mode,
                "GUARDIAN_PERCEPTION_PARAMS_FILE": _runtime_mode_config.perception_config,
                "GUARDIAN_BRIDGE_MODE": _runtime_mode_config.bridge_mode,
                "GUARDIAN_BRIDGE_MANAGER_URL": os.environ.get("GUARDIAN_BRIDGE_MANAGER_URL") or
                "http://127.0.0.1:%d/api/internal/bridge" % GUARDIAN_API_PORT,
                "GUARDIAN_BRIDGE_TOKEN": GUARDIAN_BRIDGE_TOKEN,
            })
            result = subprocess.run(
                [LIDAR_SCRIPT],
                capture_output=True,
                text=True,
                timeout=30.0,
                check=False,
                env=child_env,
            )
            if result.returncode == 0:
                log("LIDAR", "LiDAR startup script executed successfully.")
                return True
            else:
                log(
                    "LIDAR ERROR",
                    f"Script exited with code {result.returncode}. Stderr: {result.stderr.strip()}"
                )
                return False
        except subprocess.TimeoutExpired:
            log("LIDAR ERROR", f"Execution timed out for {LIDAR_SCRIPT}.")
            return False
        except Exception as e:
            log("LIDAR ERROR", f"Failed to execute script: {e}")
            return False

    # 2. Fallback: Directly verify/start the Docker container
    log(
        "LIDAR",
        f"Script not found at '{LIDAR_SCRIPT}'. Directly verifying Docker container '{DOCKER_CONTAINER_NAME}'..."
    )

    if dry_run:
        log("LIDAR DRY-RUN", f"Would ensure container '{DOCKER_CONTAINER_NAME}' is running.")
        return True

    try:
        # Check if container is already running
        check_cmd = ["docker", "ps", "-q", "-f", f"name=^{DOCKER_CONTAINER_NAME}$"]
        res_check = subprocess.run(check_cmd, capture_output=True, text=True, check=False)

        if res_check.returncode == 0 and res_check.stdout.strip():
            log("LIDAR", f"Container '{DOCKER_CONTAINER_NAME}' is already running.")
            return True

        # Container is not running, start it
        log("LIDAR", f"Container '{DOCKER_CONTAINER_NAME}' is stopped. Starting it now...")
        docker_cmd = ["sudo", "docker"] if USE_SUDO_DOCKER else ["docker"]
        start_cmd = docker_cmd + ["start", DOCKER_CONTAINER_NAME]
        res_start = subprocess.run(start_cmd, capture_output=True, text=True, check=False)

        if res_start.returncode == 0:
            log("LIDAR", f"Container '{DOCKER_CONTAINER_NAME}' started successfully.")
            return True
        else:
            log(
                "LIDAR ERROR",
                f"Failed to start container '{DOCKER_CONTAINER_NAME}'. Stderr: {res_start.stderr.strip()}"
            )
            return False

    except Exception as e:
        log("LIDAR ERROR", f"Failed to check/start container '{DOCKER_CONTAINER_NAME}': {e}")
        return False


def ensure_rviz2_running(dry_run: bool = False) -> bool:
    """Start RViz2 in the LiDAR container only when it is not already running."""
    if not RVIZ_AUTO_START:
        return True
    if _ros_process_running(RVIZ_PROCESS):
        return True
    if not _container_running():
        log("RVIZ ERROR", f"Container '{DOCKER_CONTAINER_NAME}' is not running.")
        return False
    docker_cmd = ["sudo", "docker"] if USE_SUDO_DOCKER else ["docker"]
    command = docker_cmd + [
        "exec", "-d", "-e", "DISPLAY=:0", DOCKER_CONTAINER_NAME, "bash", "-lc",
        "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && exec rviz2 -d "
        + shlex.quote(_runtime_mode_config.rviz_config),
    ]
    if dry_run:
        log("RVIZ DRY-RUN", f"Would start RViz2: {' '.join(command)}")
        return True
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10.0, check=False)
        if result.returncode == 0:
            log("RVIZ", "RViz2 started in LiDAR container.")
            return True
        log("RVIZ ERROR", result.stderr.strip() or "RViz2 start failed")
    except Exception as exc:
        log("RVIZ ERROR", str(exc))
    return False


def stop_lidar_ros2_bringup(dry_run: bool = False) -> bool:
    """Stop only the detached LiDAR launch process, leaving Docker available."""
    if not LIDAR_STOP_ON_MANAGER_STOP:
        log("LIDAR", "LiDAR ROS2 stop on Manager shutdown is disabled.")
        return True
    docker_cmd = ["sudo", "docker"] if USE_SUDO_DOCKER else ["docker"]
    # The bracket prevents pkill from matching its own command line.
    pattern = LIDAR_BRINGUP_PROCESS.replace("ros2", "[r]os2", 1)
    command = docker_cmd + ["exec", DOCKER_CONTAINER_NAME, "bash", "-lc", f"pkill -TERM -f '{pattern}' || true"]
    if dry_run:
        log("LIDAR DRY-RUN", f"Would stop LiDAR ROS2 bringup: {' '.join(command)}")
        return True
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10.0, check=False)
        if result.returncode == 0:
            log("LIDAR", "LiDAR ROS2 bringup stopped for Manager shutdown/restart.")
            return True
        log("LIDAR ERROR", f"Could not stop LiDAR ROS2 bringup: {result.stderr.strip()}")
        return False
    except Exception as exc:
        log("LIDAR ERROR", f"Could not stop LiDAR ROS2 bringup: {exc}")
        return False


# ==============================================================================
# 2. DOCKER ROS 2 COMMAND EXECUTION HELPER
# ==============================================================================
def execute_ros2_command(command: str, dry_run: bool = False, timeout: float = 10.0) -> dict:
    """
    Executes a ROS 2 command non-interactively inside the Docker container.
    Constructs:
        [sudo] docker exec <CONTAINER> bash -c "source <ROS_SETUP> && <COMMAND>"
    """
    docker_cmd = ["sudo", "docker"] if USE_SUDO_DOCKER else ["docker"]
    full_cmd = docker_cmd + [
        "exec",
        DOCKER_CONTAINER_NAME,
        "bash",
        "-c",
        f"source {ROS_SETUP_SCRIPT} && {command}"
    ]

    cmd_str = " ".join(full_cmd)

    if dry_run:
        log("ROS2 DRY-RUN", f"Command to execute: {cmd_str}")
        return {
            "success": True,
            "returncode": 0,
            "stdout": "",
            "stderr": "",
            "dry_run": True
        }

    try:
        result = subprocess.run(
            full_cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False
        )
        return {
            "success": result.returncode == 0,
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "dry_run": False
        }
    except subprocess.TimeoutExpired:
        log("ROS2 ERROR", f"Command timed out after {timeout}s: {command}")
        return {
            "success": False,
            "returncode": -1,
            "stdout": "",
            "stderr": f"Timeout after {timeout}s",
            "dry_run": False
        }
    except Exception as e:
        log("ROS2 ERROR", f"Command execution failed: {e}")
        return {
            "success": False,
            "returncode": -1,
            "stdout": "",
            "stderr": str(e),
            "dry_run": False
        }


# ==============================================================================
# 3. HOST SHELL / SUDO COMMAND EXECUTION HELPER
# ==============================================================================
def execute_host_command(command: str, dry_run: bool = False, timeout: float = 15.0) -> dict:
    """
    Executes an arbitrary shell / host command (e.g. 'sudo shutdown -h now').
    Supports sudo commands, pipes, and shell syntax.
    """
    if dry_run:
        log("HOST DRY-RUN", f"Command to execute: {command}")
        return {
            "success": True,
            "returncode": 0,
            "stdout": "",
            "stderr": "",
            "dry_run": True
        }

    try:
        log("HOST", f"Executing host command: {command}")
        result = subprocess.run(
            command,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False
        )
        return {
            "success": result.returncode == 0,
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "dry_run": False
        }
    except subprocess.TimeoutExpired:
        log("HOST ERROR", f"Command timed out after {timeout}s: {command}")
        return {
            "success": False,
            "returncode": -1,
            "stdout": "",
            "stderr": f"Timeout after {timeout}s",
            "dry_run": False
        }
    except Exception as e:
        log("HOST ERROR", f"Host command failed: {e}")
        return {
            "success": False,
            "returncode": -1,
            "stdout": "",
            "stderr": str(e),
            "dry_run": False
        }


# ==============================================================================
# 4. UPS STATUS PARSING & MONITORING
# ==============================================================================
def read_ups_status() -> tuple[dict | None, str | None]:
    """
    Reads and parses UPS metrics using apcaccess.
    Returns:
        (data_dict, None) on success:
            data_dict = {
                "status": str (e.g. "ONLINE", "ONBATT"),
                "bcharge": float | None,
                "timeleft": float | None
            }
        (None, error_str) on failure.
    """
    try:
        result = subprocess.run(
            ["apcaccess"],
            capture_output=True,
            text=True,
            timeout=2.5,
            check=False
        )
        if result.returncode != 0:
            err_msg = result.stderr.strip() or f"process exit code {result.returncode}"
            return None, f"apcaccess returned non-zero ({err_msg})"

        data: dict[str, str] = {}
        for line in result.stdout.splitlines():
            if ":" not in line:
                continue
            key, val = line.split(":", 1)
            data[key.strip()] = val.strip()

        raw_status = data.get("STATUS")
        if not raw_status:
            return None, "STATUS field not found in apcaccess output"

        # Keep the full status so a combined value cannot be mistaken for ONLINE.
        status = raw_status

        # Parse battery charge percentage
        bcharge: float | None = None
        raw_bcharge = data.get("BCHARGE")
        if raw_bcharge:
            try:
                bcharge = float(raw_bcharge.split()[0])
            except (ValueError, IndexError):
                bcharge = None

        # Parse estimated time left in minutes
        timeleft: float | None = None
        raw_timeleft = data.get("TIMELEFT")
        if raw_timeleft:
            try:
                timeleft = float(raw_raw := raw_timeleft.split()[0])
            except (ValueError, IndexError):
                timeleft = None

        return {
            "status": status,
            "bcharge": bcharge,
            "timeleft": timeleft
        }, None

    except FileNotFoundError:
        return None, "'apcaccess' executable not found in PATH"
    except subprocess.TimeoutExpired:
        return None, "apcaccess command timed out"
    except Exception as e:
        return None, str(e)


def on_ups_timeleft_low(ups: dict) -> dict:
    """
    Callback triggered when UPS TIMELEFT drops to or below HOST_SHUTDOWN_TIMELEFT_MIN.
    1. Executes UPS_TIMELEFT_ACTION_COMMAND inside Docker to verify/manipulate ROS 2.
    2. If SHUTDOWN_ENABLED is True, executes host shutdown command (e.g. 'sudo shutdown -h now').
    """
    global _timeleft_action_triggered
    _timeleft_action_triggered = True

    status = ups.get("status", "UNKNOWN")
    timeleft = ups.get("timeleft")
    bcharge = ups.get("bcharge")

    log(
        "UPS TRIGGER",
        f"TIMELEFT ({timeleft} min) <= threshold ({HOST_SHUTDOWN_TIMELEFT_MIN} min) "
        f"[STATUS={status}, BCHARGE={bcharge}%]. Triggering Docker ROS 2 action..."
    )

    # 1. Execute Docker ROS 2 command
    ros2_result = execute_ros2_command(
        UPS_TIMELEFT_ACTION_COMMAND,
        dry_run=UPS_TIMELEFT_ACTION_DRY_RUN
    )

    if ros2_result.get("dry_run"):
        log("ROS2 ACTION", "Dry-run mode: command verified without live execution.")
    elif ros2_result.get("success"):
        stdout_preview = ros2_result.get("stdout", "").strip()
        log(
            "ROS2 SUCCESS",
            f"Command successfully reached and executed inside Docker '{DOCKER_CONTAINER_NAME}'.\n"
            f"--- Command Output ---\n{stdout_preview}\n----------------------"
        )
    else:
        stderr_preview = ros2_result.get("stderr", "").strip()
        log(
            "ROS2 ERROR",
            f"Command failed inside Docker '{DOCKER_CONTAINER_NAME}'. Code: {ros2_result.get('returncode')}\n"
            f"--- Error Output ---\n{stderr_preview}\n--------------------"
        )

    # 2. Execute Host Shutdown / Arbitrary Sudo Command (if enabled)
    shutdown_result = None
    if SHUTDOWN_ENABLED:
        if SHUTDOWN_DELAY_SEC > 0:
            log("SHUTDOWN", f"Waiting {SHUTDOWN_DELAY_SEC}s before executing shutdown command...")
            time.sleep(SHUTDOWN_DELAY_SEC)

        log("SHUTDOWN", f"Triggering host shutdown command: '{SHUTDOWN_COMMAND}'")
        shutdown_result = execute_host_command(
            SHUTDOWN_COMMAND,
            dry_run=SHUTDOWN_DRY_RUN
        )

        if shutdown_result.get("dry_run"):
            log("SHUTDOWN", "Dry-run mode active: Host was NOT shut down (safe test mode).")
        elif shutdown_result.get("success"):
            log("SHUTDOWN SUCCESS", "Host shutdown command executed successfully.")
        else:
            log(
                "SHUTDOWN ERROR",
                f"Host shutdown command failed with code {shutdown_result.get('returncode')}. "
                f"Stderr: {shutdown_result.get('stderr', '').strip()}"
            )
    else:
        log("SHUTDOWN", "Shutdown is disabled (SHUTDOWN_ENABLED=False). Host remains running.")

    return {
        "ros2": ros2_result,
        "shutdown": shutdown_result
    }


def monitor_ups_step() -> float:
    """
    Performs one cycle of UPS health check.
    Returns the next sleep interval (seconds):
        - UPS_ONLINE_CHECK_INTERVAL (30.0s) when STATUS == ONLINE
        - UPS_ALERT_CHECK_INTERVAL (5.0s) for all other statuses or read errors
    """
    global _last_ups
    ups, error = read_ups_status()

    # 1. APCACCESS Execution / Parsing Failure
    if error is not None or ups is None:
        _last_ups = {"enabled": True, "error": error or "unknown"}
        log("UPS ERROR", f"Unable to read UPS status: {error or 'unknown'}")
        return UPS_ALERT_CHECK_INTERVAL

    status = ups["status"]
    timeleft = ups.get("timeleft")
    bcharge = ups.get("bcharge")
    _last_ups = {
        "enabled": True, "status": status, "timeleft_min": timeleft, "bcharge": bcharge,
        "lidar_protection_enabled": _runtime_mode_config.lidar_ups_protection_enabled,
        "host_shutdown_enabled": False,
    }

    if status == "ONLINE":
        if _runtime_mode_config.lidar_ups_protection_enabled:
            log("UPS", "STATUS=ONLINE -> requesting LiDAR NORMAL")
            result = command_lidar("normal")
            if result.get("success"):
                log("LIDAR UPS", "CX128S2 NORMAL command succeeded")
            else:
                log("LIDAR UPS ERROR", f"NORMAL command failed: {result.get('error', 'unknown error')}")
        return UPS_ONLINE_CHECK_INTERVAL

    if status == "ONBATT" and _runtime_mode_config.lidar_ups_protection_enabled:
        log("UPS", "STATUS=ONBATT -> requesting LiDAR STANDBY")
        result = command_lidar("standby", LIDAR_MESSAGE_UPS_LOW_BATTERY)
        if result.get("success"):
            log("LIDAR UPS", "CX128S2 STANDBY command succeeded")
        else:
            log("LIDAR UPS ERROR", f"STANDBY command failed: {result.get('error', 'unknown error')}")

    bcharge_str = f"{bcharge}%" if bcharge is not None else "N/A"
    timeleft_str = f"{timeleft} min" if timeleft is not None else "N/A"
    tag = "UPS" if status == "ONBATT" else "UPS WARNING"
    log(tag, f"STATUS={status} | BCHARGE={bcharge_str} | TIMELEFT={timeleft_str}")
    return UPS_ALERT_CHECK_INTERVAL


# ==============================================================================
# MAIN ENTRYPOINT
# ==============================================================================
def _build_lidar_controller():
    return CX128S2Controller(
        lidar_ip=CX128S2_IP,
        interface=CX128S2_INTERFACE,
        cache_file=CX128S2_CACHE_FILE,
    )


def _parse_args():
    parser = argparse.ArgumentParser(description="Guardian AI Manager")
    parser.add_argument("mode", nargs="?", choices=tuple(MODE_CONFIG),
                        help=f"system profile (default: {SYSTEM_MODE_DEFAULT})")
    parser.add_argument("--mode", dest="mode_option", choices=tuple(MODE_CONFIG),
                        help="system profile (compatible with existing service commands)")
    commands = parser.add_mutually_exclusive_group()
    commands.add_argument("--lidar-standby", action="store_true", help="send cached CX128S2 standby command")
    commands.add_argument("--lidar-normal", action="store_true", help="send cached CX128S2 normal command")
    commands.add_argument("--lidar-status", action="store_true", help="show CX128S2 controller status")
    commands.add_argument("--lidar-capture-reference", action="store_true", help="optionally cache a normal UCWP frame from live CX128S2 DIFOP")
    args = parser.parse_args()
    if args.mode is not None and args.mode_option is not None:
        parser.error("specify the mode either positionally or with --mode")
    args.mode = args.mode or args.mode_option or SYSTEM_MODE_DEFAULT
    return args


def main():
    global _lidar_controller, _packet_monitor, _redis_publisher, _runtime_mode_config, _last_ups
    args = _parse_args()
    _runtime_mode_config = MODE_CONFIG[args.mode]
    _last_ups = {"enabled": _runtime_mode_config.ups_monitor_enabled}
    if args.lidar_standby or args.lidar_normal or args.lidar_status or args.lidar_capture_reference:
        controller = _build_lidar_controller()
        if args.lidar_standby:
            result = controller.standby()
        elif args.lidar_normal:
            result = controller.normal()
        elif args.lidar_capture_reference:
            result = controller.capture_reference_packet()
        else:
            result = controller.status()
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["success"] else 1

    # Register signal handlers for clean exit
    signal.signal(signal.SIGINT, handle_shutdown_signal)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)

    print("=" * 65)
    print(" Guardian AI Manager Starting")
    print("=" * 65)
    log("CONFIG", f"LIDAR_AUTO_START           = {LIDAR_AUTO_START}")
    log("CONFIG", f"LIDAR_SCRIPT               = {LIDAR_SCRIPT}")
    log("CONFIG", f"LIDAR_STOP_ON_MANAGER_STOP = {LIDAR_STOP_ON_MANAGER_STOP}")
    log("CONFIG", f"RVIZ_AUTO_START            = {RVIZ_AUTO_START}")
    log("CONFIG", f"ROS2_ENTER_SCRIPT          = {ROS2_ENTER_SCRIPT}")
    log("CONFIG", f"DOCKER_CONTAINER_NAME      = {DOCKER_CONTAINER_NAME}")
    log("CONFIG", f"USE_SUDO_DOCKER            = {USE_SUDO_DOCKER}")
    log("CONFIG", f"SYSTEM_MODE                = {_runtime_mode_config.mode}")
    log("CONFIG", f"PERCEPTION_CONFIG          = {_runtime_mode_config.perception_config}")
    log("CONFIG", f"BRIDGE_MODE                = {_runtime_mode_config.bridge_mode}")
    log("CONFIG", f"UPS_MONITOR_ENABLED        = {_runtime_mode_config.ups_monitor_enabled}")
    log("CONFIG", f"UPS_LIDAR_PROTECTION       = {_runtime_mode_config.lidar_ups_protection_enabled}")
    log("CONFIG", f"UPS_ONLINE_CHECK_INTERVAL  = {UPS_ONLINE_CHECK_INTERVAL}s")
    log("CONFIG", f"UPS_ALERT_CHECK_INTERVAL   = {UPS_ALERT_CHECK_INTERVAL}s")
    log("CONFIG", f"HOST_SHUTDOWN_TIMELEFT_MIN = {HOST_SHUTDOWN_TIMELEFT_MIN} min")
    log("CONFIG", f"UPS_TIMELEFT_ACTION_CMD    = {UPS_TIMELEFT_ACTION_COMMAND}")
    log("CONFIG", f"UPS_TIMELEFT_ACTION_DRY_RUN= {UPS_TIMELEFT_ACTION_DRY_RUN}")
    log("CONFIG", "SHUTDOWN_ENABLED           = False (all supported modes)")
    log("CONFIG", f"SHUTDOWN_COMMAND           = {SHUTDOWN_COMMAND}")
    log("CONFIG", f"SHUTDOWN_DRY_RUN           = {SHUTDOWN_DRY_RUN}")
    log("CONFIG", f"SHUTDOWN_DELAY_SEC         = {SHUTDOWN_DELAY_SEC}s")
    log("CONFIG", f"CX128S2_ENABLED            = {CX128S2_ENABLED}")
    log("CONFIG", f"CX128S2_IP                 = {CX128S2_IP}")
    log("CONFIG", f"CX128S2_INTERFACE          = {CX128S2_INTERFACE or 'auto'}")
    log("CONFIG", f"CX128S2_CACHE_FILE         = {CX128S2_CACHE_FILE}")
    log("CONFIG", f"GUARDIAN_API_BIND           = {GUARDIAN_API_BIND}")
    log("CONFIG", f"GUARDIAN_API_PORT           = {GUARDIAN_API_PORT}")
    log("CONFIG", f"REDIS_ENABLED               = {REDIS_ENABLED}")
    log("CONFIG", f"REDIS_PUBLISH_INTERVAL_SEC  = {REDIS_PUBLISH_INTERVAL_SEC}s")
    print("=" * 65)

    # 1. LiDAR Docker auto-start
    start_lidar_docker()
    ensure_rviz2_running()

    # 2. ROS 2 command bridge readiness check (dry-run)
    execute_ros2_command("ros2 topic list --no-daemon", dry_run=True)

    # 3. Host-side LiDAR control API. It does not use Docker or ROS commands.
    if CX128S2_ENABLED:
        _lidar_controller = _build_lidar_controller()
        status = _lidar_controller.status()
        log("LIDAR CTRL", "CX128S2 controller initialized")
        log("LIDAR CTRL", f"LiDAR: {CX128S2_IP}")
        log("LIDAR CTRL", f"Interface: {status.get('interface') or status.get('error', 'unavailable')}")
        log("LIDAR CTRL", f"Valid UCWP cache {'loaded' if status['cached_packet'] else 'not found'}")
        _packet_monitor = CX128S2PacketMonitor(
            CX128S2_IP, CX128S2_INTERFACE, CX128S2_MSOP_PORT, CX128S2_PACKET_TIMEOUT_SEC
        )
        _packet_monitor.start()
        log("LIDAR CTRL", f"MSOP monitor started (timeout={CX128S2_PACKET_TIMEOUT_SEC}s)")
        start_lidar_api(_lidar_controller)

    _redis_publisher = RedisPublisher(
        REDIS_ENABLED, REDIS_HOST, REDIS_PORT,
        REDIS_CRANE_KEY if _runtime_mode_config.mode == "crane" else REDIS_KEY, logger=log
    )

    # 4. UPS monitoring and API lifetime loop.
    if not _runtime_mode_config.ups_monitor_enabled:
        log("UPS", "Monitoring is disabled for general mode. API remains active.")
    else:
        log("UPS", "Starting UPS monitor loop. Press Ctrl+C to terminate.")

    next_ups_check = 0.0
    next_redis_publish = 0.0
    next_capture_check = 0.0
    next_runtime_check = time.monotonic() + LIDAR_RUNTIME_CHECK_INTERVAL_SEC
    while _running:
        refresh_lidar_health()
        now = time.monotonic()
        requested_state, _ = _external_state().lidar_context()
        if requested_state == "normal" and now >= next_runtime_check:
            # Recover only missing container/ROS/RViz processes. A running
            # driver with no MSOP remains a packet-health fault, not a blind
            # restart loop.
            start_lidar_docker()
            ensure_rviz2_running()
            next_runtime_check = now + LIDAR_RUNTIME_CHECK_INTERVAL_SEC
        if now >= next_capture_check:
            if _runtime_mode_config.mode == "general" and GENERAL_AUTO_CAPTURE_ENABLED:
                threading.Thread(target=_capture_manager.collect_completed, daemon=True, name="capture-collection").start()
                threading.Thread(target=_capture_manager.capture_crane, daemon=True, name="crane-capture").start()
            next_capture_check = now + 10.0
        if _runtime_mode_config.mode == "general":
            bridge, fresh = _bridge_state.snapshot(BRIDGE_HEALTH_TIMEOUT_SEC)
            packet_healthy = _packet_monitor is not None and _packet_monitor.health()[0]
            operating = (requested_state == "normal" and packet_healthy and fresh
                         and bridge.get("lidar", {}).get("pointcloud_active", False)
                         and bridge.get("lidar", {}).get("perception_active", False)
                         and bridge.get("perception", {}).get("vehicle_count", 0) > 0)
            if not operating:
                _capture_manager._general_started_at = None
            elif _capture_manager._general_started_at is None:
                _capture_manager._general_started_at = now
                _capture_manager._general_alert = None
            elif now - _capture_manager._general_started_at >= GENERAL_ALERT_AFTER_SEC and not _capture_manager._general_alert:
                capture_id = _capture_manager.start_rosbag() if GENERAL_AUTO_CAPTURE_ENABLED else None
                _capture_manager._general_alert = {"message": "General vehicle recognition has continued for 3 hours",
                                                    "timestamp_ms": int(time.time() * 1000), "capture_id": capture_id}
                log("LIDAR OPERATION ALERT", _capture_manager._general_alert["message"])
        if _runtime_mode_config.ups_monitor_enabled and now >= next_ups_check:
            next_ups_check = now + monitor_ups_step()
        if now >= next_redis_publish or _external_state().consume_changed():
            _redis_publisher.publish(build_external_payload())
            next_redis_publish = now + REDIS_PUBLISH_INTERVAL_SEC
        time.sleep(0.2)

    log("MANAGER", "Guardian AI Manager stopped cleanly.")
    stop_lidar_api()
    if _packet_monitor is not None:
        _packet_monitor.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
