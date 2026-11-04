#!/usr/bin/env python3
"""External desktop/terminal client for existing Guardian APIs."""
import argparse
import csv
import io
import json
import os
import queue
import re
import shutil
import shlex
import subprocess
import tarfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import unicodedata
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path


# Default local storage follows the script when it is copied to another PC.
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "lidar_collections"


def now_text():
    return datetime.now().astimezone().isoformat(timespec="seconds")


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


@dataclass
class Target:
    name: str
    url: str
    ssh: str = ""
    status: dict = field(default_factory=dict)
    captures: dict = field(default_factory=dict)
    connected: bool = False
    checked_at: float = 0.0
    error: str = ""
    phase: str = "연결 대기"
    capture_id: str = ""
    uncertain: bool = False
    repeat: bool = False
    collect: bool = False
    list_supported: object = None
    busy: bool = False
    operated_since: object = None
    alert_sent: bool = False
    diagnostic_pending: bool = False
    auto_diagnostic: bool = False
    sessions_pending: bool = False
    actions: queue.Queue = field(default_factory=queue.Queue)


class ControlCenter:
    """One independent polling worker per computer; GUI only consumes snapshots."""
    def __init__(self, general_url, crane_url, output, poll=5.0, demo=False):
        self.output = Path(output).expanduser().resolve()
        self.poll = poll
        self.demo = demo
        self.targets = {name: Target(name, url.rstrip("/")) for name, url in
                        (("general", general_url), ("crane", crane_url))}
        self.events = deque(maxlen=1000)
        self.event_counter = 0
        self.tracks = {}
        self.track_times = {}
        self.track_received = 0.0
        self.track_source = "실시간 추적 정보 없음 — HTTP는 요약만 제공"
        self.ros_status = "ROS 구독 꺼짐"
        self.ros_details = {}
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.threads = []
        self.started = False
        self.ros_context = None
        for target in self.targets.values():
            checkpoint = self.folder(target) / "pending.json"
            if checkpoint.is_file():
                saved = json.loads(checkpoint.read_text())
                if saved.get("url") != target.url:
                    target.error = "미완료 작업의 주소가 다릅니다. pending.json을 확인하세요."
                    target.uncertain = True
                else:
                    target.capture_id = saved.get("capture_id", "")
                    target.uncertain = saved.get("uncertain", False)

            # Restore the local file view even when a legacy server has no list API.
            for metadata in (self.folder(target) / "captures").glob("*/download.json"):
                try:
                    saved = json.loads(metadata.read_text())
                    capture_id = metadata.parent.name
                    if re.fullmatch(r"[A-Za-z0-9_]+", capture_id) and (metadata.parent / "original.tar.gz").is_file():
                        target.captures[capture_id] = {**saved.get("job", {}), "capture_id": capture_id,
                                                       "state": "complete", "size_bytes": saved.get("size_bytes")}
                except (OSError, ValueError, TypeError):
                    continue

    def folder(self, target):
        return self.output / target.name

    def log(self, target, category, message, **detail):
        event = {"time": now_text(), "target": target.name if target else "client",
                 "category": category, "message": message, **detail}
        with self.lock:
            self.event_counter += 1
            event["sequence"] = self.event_counter
            self.events.append(event)
            path = self.output / event["target"] / datetime.now().strftime("%Y-%m-%d") / "communication.jsonl"
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=False) + "\n")

    def checkpoint(self, target):
        path = self.folder(target) / "pending.json"
        if target.capture_id or target.uncertain:
            atomic_json(path, {"url": target.url, "capture_id": target.capture_id, "uncertain": target.uncertain})
        else:
            path.unlink(missing_ok=True)

    def request(self, target, path, body=None, timeout=15):
        parsed = urllib.parse.urlparse(target.url)
        if parsed.scheme not in ("http", "https") or not parsed.netloc or parsed.query or parsed.fragment:
            raise ValueError("http(s)://주소:포트 형태로 입력하세요")
        method = "POST" if body is not None else "GET"
        request = urllib.request.Request(target.url + path, method=method,
                                         data=json.dumps(body).encode() if body is not None else None,
                                         headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                value = json.load(response)
            self.log(target, "HTTP", f"{method} {path} 성공", response=value)
            return value
        except Exception as exc:
            self.log(target, "ERROR", f"{method} {path}: {exc}")
            raise

    def ready(self, target):
        return (target.connected and time.monotonic() - target.checked_at < max(30, self.poll * 3)
                and target.status.get("system_mode") == target.name)

    def crane_coordinator_conflict(self):
        general = self.targets["general"]
        if not self.ready(general):
            return "General 상태를 먼저 확인해야 기존 수집기와 중복을 피할 수 있습니다."
        coordinator = general.status.get("crane_capture") or {}
        if coordinator.get("manager_url"):
            return "General Manager의 기존 Crane 수집기가 활성 상태입니다. 외부 자동 수집은 대기합니다."
        return ""

    def start_capture(self, target, duration):
        if not self.ready(target):
            raise RuntimeError("연결 상태와 컴퓨터 모드를 먼저 확인하세요")
        if target.capture_id or target.uncertain:
            raise RuntimeError("진행 중인 작업 또는 결과 불명 요청이 있습니다")
        if target.name == "crane":
            conflict = self.crane_coordinator_conflict()
            if conflict:
                raise RuntimeError(conflict)
        if not target.status.get("bridge_connected") or not target.status.get("lidar", {}).get("pointcloud_active"):
            raise RuntimeError("PointCloud가 활성 상태가 아닙니다. Recorder 서비스 준비는 기존 API로 별도 확인할 수 없습니다.")
        if isinstance(duration, bool) or not isinstance(duration, int) or not 1 <= duration <= 120:
            raise ValueError("녹화 시간은 1~120초 정수입니다")
        self.folder(target).mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(self.folder(target)).free < 1024 ** 3:
            raise RuntimeError("다운로드 저장소 여유 공간이 1GiB 미만입니다")
        target.uncertain = True
        target.phase = "녹화 시작 요청"
        self.checkpoint(target)
        self.log(target, "CAPTURE", f"{duration}초 녹화 요청")
        try:
            result = self.request(target, "/api/lidar/capture", {"duration_sec": duration}, timeout=40)
        except urllib.error.HTTPError as exc:
            if 400 <= exc.code < 500:
                target.uncertain = False
                self.checkpoint(target)
            raise
        except urllib.error.URLError as exc:
            if isinstance(exc.reason, OSError) and exc.reason.errno in (111, 101, 113):
                target.uncertain = False
                self.checkpoint(target)
            raise
        if result.get("success") is not True:
            target.uncertain = False
            self.checkpoint(target)
            raise RuntimeError(result.get("error", "시작 실패"))
        capture_id = result.get("capture_id", "")
        if not isinstance(capture_id, str) or not re.fullmatch(r"[A-Za-z0-9_]+", capture_id):
            raise RuntimeError("응답의 작업 ID가 올바르지 않습니다")
        target.capture_id = capture_id
        target.uncertain = False
        self.checkpoint(target)
        target.phase = "녹화 요청 수락 · 완료 대기"
        self.log(target, "CAPTURE", "녹화 요청 수락", capture_id=capture_id)

    def download(self, target, capture_id, job):
        if not re.fullmatch(r"[A-Za-z0-9_]+", capture_id):
            raise ValueError("잘못된 작업 ID")
        folder = self.folder(target) / "captures" / capture_id
        destination = folder / "original.tar.gz"
        if (folder / "download.json").is_file() and destination.is_file():
            return destination
        folder.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(folder).free < max(1024 ** 3, int(job.get("size_bytes") or 0) * 2):
            raise RuntimeError("다운로드/분류용 디스크 공간이 부족합니다")
        temporary = folder / "original.tar.gz.part"
        target.phase = "파일 다운로드"
        self.log(target, "DOWNLOAD", "다운로드 시작", capture_id=capture_id)
        with urllib.request.urlopen(target.url + "/api/lidar/capture/" + capture_id + "/download", timeout=60) as response:
            expected = int(response.headers.get("Content-Length", "0"))
            received = 0
            with temporary.open("wb") as stream:
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    stream.write(chunk)
                    received += len(chunk)
                    target.phase = f"다운로드 {received / 1024 ** 2:.1f} / {expected / 1024 ** 2:.1f} MiB"
            if not expected or received != expected or (job.get("size_bytes") and received != job["size_bytes"]):
                raise RuntimeError("다운로드 크기가 맞지 않습니다. 같은 ID로 재시도합니다.")
        self.inspect_archive(temporary, folder, target)
        temporary.replace(destination)
        atomic_json(folder / "download.json", {"time": now_text(), "url": target.url,
                                               "capture_id": capture_id, "job": job, "size_bytes": received})
        self.log(target, "DOWNLOAD", "다운로드 및 분류 완료", capture_id=capture_id, file=str(destination))
        target.phase = "다운로드 완료"
        return destination

    def inspect_archive(self, source, folder, target):
        """Never extract remote paths; copy small metadata and create a raw-only 1Hz PCD archive."""
        with tarfile.open(source, "r:gz") as archive:
            members = {member.name.removeprefix("./"): member for member in archive.getmembers()}
            if any(name.startswith("/") or ".." in Path(name).parts or member.issym() or member.islnk()
                   for name, member in members.items()):
                raise RuntimeError("압축 파일에 안전하지 않은 경로가 있습니다")
            inventory = [{"name": name, "size": member.size} for name, member in members.items() if member.isfile()]
            atomic_json(folder / "inventory.json", inventory)
            if "session.json" not in members:
                if "metadata.yaml" not in members or not any(name.endswith((".mcap", ".db3")) for name in members):
                    raise RuntimeError("알려진 PCD 세션 또는 rosbag 형식이 아닙니다")
                metadata = archive.extractfile(members["metadata.yaml"]).read(1024 * 1024)
                messages = re.search(rb"\bmessage_count:\s*(\d+)", metadata)
                if not messages or int(messages.group(1)) == 0:
                    raise RuntimeError("bag에 기록된 메시지가 없습니다")
                (folder / "bag_metadata.yaml").write_bytes(metadata)
                atomic_json(folder / "summary.json", {"kind": "rosbag", "files": len(inventory)})
                return
            metadata_member = members["session.json"]
            if metadata_member.size > 8 * 1024 * 1024:
                raise RuntimeError("세션 메타데이터가 너무 큽니다")
            session = json.load(archive.extractfile(metadata_member))
            if session.get("complete") is False or session.get("success") is False:
                raise RuntimeError("서버가 미완료/실패로 표시한 세션입니다")
            if "complete" not in session and not session.get("event_end_stamp_ns"):
                raise RuntimeError("구버전 세션의 종료 timestamp가 없습니다")
            atomic_json(folder / "session.json", session)
            if "frames.csv" not in members:
                raise RuntimeError("frames.csv가 없습니다")
            if members["frames.csv"].size > 32 * 1024 * 1024:
                raise RuntimeError("프레임 인덱스가 너무 큽니다")
            lines = archive.extractfile(members["frames.csv"]).read().decode().splitlines()
            header = next(csv.reader([lines[0]]))
            rows, selected, seconds = [], [], set()
            for line in lines[1:]:
                values = next(csv.reader([line + ('"' if line.endswith(',"') else '')]))
                if len(values) != len(header):
                    raise RuntimeError("프레임 인덱스 형식이 올바르지 않습니다")
                row = dict(zip(header, values))
                raw = row.get("raw_pcd_path", "")
                if "WRITE_FAILED" in row.get("save_status", "") or "DISK_LOW" in row.get("save_status", ""):
                    raise RuntimeError("서버가 프레임 저장 실패를 기록했습니다")
                if not raw or raw not in members or not members[raw].isfile() or not members[raw].size:
                    raise RuntimeError("프레임의 원본 PCD가 누락됐습니다")
                rows.append(row)
                if row["stamp_sec"] not in seconds:
                    seconds.add(row["stamp_sec"])
                    selected.append(row)
            if not rows:
                raise RuntimeError("저장된 PCD가 없습니다")
            with (folder / "frames.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=header)
                writer.writeheader()
                writer.writerows(rows)
            if target.name == "crane":
                sample = folder / "raw_1hz.tar.gz.part"
                text = io.StringIO()
                writer = csv.DictWriter(text, fieldnames=header)
                writer.writeheader()
                writer.writerows(selected)
                with tarfile.open(sample, "w:gz") as sampled:
                    for row in selected:
                        member = members[row["raw_pcd_path"]]
                        sampled.addfile(member, archive.extractfile(member))
                    for name, payload in (("frames.csv", text.getvalue().encode()),
                                          ("session.json", json.dumps({**session, "selection": "first raw frame per stamp_sec",
                                                                       "selected_frames": len(selected)}).encode())):
                        info = tarfile.TarInfo(name)
                        info.size = len(payload)
                        sampled.addfile(info, io.BytesIO(payload))
                sample.replace(folder / "raw_1hz.tar.gz")
            objects = []
            for name, member in members.items():
                if name.startswith("annotations/") and name.endswith(".json") and member.size < 1024 * 1024:
                    annotation = json.load(archive.extractfile(member))
                    for obj in annotation.get("objects", []):
                        objects.append({"source": name, **obj})
                        if len(objects) >= 5000:
                            break
                if len(objects) >= 5000:
                    break
            atomic_json(folder / "objects.json", objects)
            atomic_json(folder / "summary.json", {"kind": "pcd", "frames": len(rows), "one_hz_frames": len(selected),
                                                    "raw_frame": session.get("raw_frame_id"),
                                                    "objects": len(objects), "files": len(inventory)})

    def capture_step(self, target):
        if target.uncertain:
            target.phase = "시작 결과 불명 — 작업 ID 확인 필요"
            return
        if target.capture_id:
            job = self.request(target, "/api/lidar/capture/" + target.capture_id)
            target.captures[target.capture_id] = job
            state = job.get("state", "unknown")
            target.phase = {"recording": "서버 작업 진행 · 파일 완료 미확인", "archiving": "압축 진행", "complete": "다운로드 준비 확인"}.get(state, state)
            if state == "failed":
                target.capture_id = ""
                self.checkpoint(target)
                raise RuntimeError(job.get("error", "원격 녹화 실패"))
            if state == "complete" and job.get("archive_path") and job.get("size_bytes"):
                self.download(target, target.capture_id, job)
                target.capture_id = ""
                self.checkpoint(target)
            return
        if target.repeat and target.name == "crane":
            self.start_capture(target, 60)

    def poll_target(self, target):
        previous_check = target.checked_at
        try:
            status = self.request(target, "/api/lidar/status")
        except Exception:
            target.connected = False
            target.operated_since = None
            target.alert_sent = False
            raise
        if previous_check and time.monotonic() - previous_check > max(15, self.poll * 3):
            target.operated_since = None
            target.alert_sent = False
        if not isinstance(status, dict):
            raise RuntimeError("상태 응답이 JSON 객체가 아닙니다")
        target.status = status
        target.checked_at = time.monotonic()
        target.connected = True
        target.error = ""
        if status.get("system_mode") != target.name:
            raise RuntimeError(f"대상 모드가 {status.get('system_mode', '미제공')}입니다. {target.name} 주소를 확인하세요.")
        if not target.capture_id and not target.repeat and not target.uncertain:
            target.phase = "모니터링"
        day = self.folder(target) / datetime.now().strftime("%Y-%m-%d") / "status"
        atomic_json(day / (datetime.now().strftime("%H%M%S_%f") + ".json"), status)
        if target.list_supported is not False:
            try:
                captures = self.request(target, "/api/lidar/capture").get("captures", [])
                target.list_supported = True
                for job in captures:
                    if re.fullmatch(r"[A-Za-z0-9_]+", str(job.get("capture_id", ""))):
                        target.captures[job["capture_id"]] = job
            except urllib.error.HTTPError as exc:
                if exc.code in (404, 405):
                    target.list_supported = False
                    self.log(target, "CAPABILITY", "구버전: 녹화 목록 API 미지원. 반환된 작업 ID로 조회합니다.")
                else:
                    raise
        self.capture_step(target)
        if target.collect:
            diagnostic = status.get("diagnostic_capture")
            if diagnostic and diagnostic.get("capture_id"):
                target.captures[diagnostic["capture_id"]] = diagnostic
            for capture_id, job in list(target.captures.items()):
                if job.get("state") == "complete" and job.get("archive_path") and job.get("size_bytes") and job.get("kind") != "ssh_session":
                    self.download(target, capture_id, job)
            if target.name == "general" and target.list_supported is False and target.ssh and not target.capture_id and not target.sessions_pending:
                target.sessions_pending = True
                thread = threading.Thread(target=self.collect_sessions, args=(target,), daemon=True)
                self.threads = [worker for worker in self.threads if worker.is_alive()]
                self.threads.append(thread)
                thread.start()
        lidar = status.get("lidar", {})
        healthy = bool(status.get("bridge_connected") and lidar.get("pointcloud_active")
                       and lidar.get("packet_active") and lidar.get("perception_active")
                       and status.get("perception", {}).get("vehicle_count", 0) > 0)
        if target.name == "general":
            if not healthy:
                target.operated_since = None
                target.alert_sent = False
            elif target.operated_since is None:
                target.operated_since = time.monotonic()
            elif time.monotonic() - target.operated_since >= 10800 and not target.alert_sent:
                target.alert_sent = True
                self.log(target, "ALERT", "General 연속 차량 인지 3시간 경과")
                if "operation_elapsed_sec" not in status and target.ssh and target.auto_diagnostic:
                    self.submit(target.name, "bag")

    def submit(self, name, action, value=None):
        target = self.targets[name]
        # Local switches work immediately, including while the network is down.
        if action in ("repeat", "collect", "auto_diagnostic"):
            setattr(target, action, bool(value))
            self.log(target, "COMMAND", f"{action}: {bool(value)}")
            return
        if not self.demo and action in ("capture", "normal", "standby", "bag") and not self.ready(target):
            target.error = "연결 상태와 컴퓨터 모드를 먼저 확인하세요"
            self.log(target, "ERROR", target.error)
            return
        target.actions.put((action, value, time.monotonic()))

    def action(self, target, action, value=None):
        if action == "capture":
            self.start_capture(target, int(value))
        elif action in ("normal", "standby"):
            if not self.ready(target):
                raise RuntimeError("대상 연결 및 모드를 먼저 확인하세요")
            result = self.request(target, "/api/lidar/" + action, {})
            if result.get("success") is not True:
                raise RuntimeError(result.get("error", "명령 실패"))
            self.log(target, "COMMAND", f"{action} 요청 수락 · 다음 상태 조회로 확인")
        elif action == "repeat":
            target.repeat = bool(value)
            if target.repeat:
                conflict = self.crane_coordinator_conflict()
                if conflict:
                    target.repeat = False
                    raise RuntimeError(conflict)
            self.log(target, "COMMAND", "Crane 반복 " + ("시작" if target.repeat else "중지 (현재 작업은 완료/회수)"))
        elif action == "collect":
            target.collect = bool(value)
        elif action == "recover":
            capture_id = str(value)
            if not re.fullmatch(r"[A-Za-z0-9_]+", capture_id):
                raise ValueError("올바른 작업 ID를 입력하세요")
            self.request(target, "/api/lidar/capture/" + capture_id)
            target.capture_id, target.uncertain = capture_id, False
            self.checkpoint(target)
            self.log(target, "CAPTURE", "작업 ID 복구", capture_id=capture_id)
        elif action in ("journal", "tracks", "bag"):
            if not target.ssh:
                raise RuntimeError("서버 전체 로그/CLI 진단은 선택적 SSH 설정이 필요합니다. HTTP에는 해당 API가 없습니다.")
            if self.demo:
                self.log(target, "DEMO", "SSH 명령을 실제로 실행하지 않습니다")
                return
            if action == "bag" and target.name != "general":
                raise RuntimeError("진단 rosbag은 General에서만 지원합니다")
            if action == "bag" and ("operation_elapsed_sec" in target.status or target.capture_id):
                raise RuntimeError("현재 General Manager가 3시간 rosbag을 관리합니다. 중복 작업을 시작하지 않습니다.")
            if target.diagnostic_pending:
                raise RuntimeError("SSH 수집이 이미 진행 중입니다")
            target.diagnostic_pending = True
            thread = threading.Thread(target=self.ssh_collect, args=(target, action), daemon=True)
            self.threads = [worker for worker in self.threads if worker.is_alive()]
            self.threads.append(thread)
            thread.start()
        else:
            raise ValueError("알 수 없는 명령")

    def ssh_collect(self, target, action):
        try:
            if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.@:-]*", target.ssh):
                raise ValueError("SSH는 user@host 형식으로 설정하세요")
            base = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "--", target.ssh]
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            if action == "journal":
                command = "journalctl -u guardian-manager.service --no-pager -o short-iso --since '1 hour ago' -n 2000"
                result = subprocess.run(base + [command], capture_output=True, timeout=30, check=True)
                path = self.folder(target) / "remote_logs" / (stamp + ".log")
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(result.stdout)
            elif action == "tracks":
                import yaml
                topic = "/tracked_objects" if target.name == "general" else "/crane_beam/status"
                message_type = ("cx128s2_bev_perception/msg/TrackedObjectArray" if target.name == "general" else
                                "cx128s2_bev_perception/msg/CraneBeamStatus")
                script = ("source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && "
                          f"timeout 8s ros2 topic echo --no-daemon --once --full-length {topic} {message_type}")
                command = "docker exec ros2_jazzy_lidar bash -lc " + shlex.quote(script)
                result = subprocess.run(base + [command], capture_output=True, timeout=20, check=True)
                value = next((doc for doc in yaml.safe_load_all(result.stdout.decode()) if isinstance(doc, dict)), None)
                if value is None:
                    raise RuntimeError("ROS 스냅샷 응답이 없습니다")
                path = self.folder(target) / "tracking_snapshots" / (stamp + ".json")
                atomic_json(path, {"time": now_text(), "topic": topic, "snapshot": value})
                with self.lock:
                    if target.name == "general":
                        objects = value.get("objects")
                        if not isinstance(objects, list):
                            raise RuntimeError("객체 배열이 없는 ROS 응답입니다")
                        key = "general:SSH " + topic
                        self.tracks[key] = objects
                        self.track_times[key] = time.monotonic()
                        self.track_received = time.monotonic()
                        self.track_source = "SSH 일회성 스냅샷 — 실시간 구독 아님"
                    else:
                        self.ros_details["crane:SSH " + topic] = value
            else:
                name = "external_bag_" + stamp
                root = "/root/ros2_ws/lidar_diagnostics"
                topics = "/ch/lslidar_point_cloud /detected_objects /tracked_objects /object_motion_states /road_vehicle_tracks /tf /tf_static"
                # All shell text is fixed; only the internally generated safe basename varies.
                script = ("source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && "
                          f"mkdir -p {root} && python3 -c \"import shutil; assert shutil.disk_usage('{root}').free > 1073741824\" && "
                          f"timeout --signal=TERM --kill-after=15s 60s ros2 bag record --disable-keyboard-controls "
                          f"-s mcap -o {root}/{name} --topics {topics}")
                command = "docker exec ros2_jazzy_lidar bash -lc " + shlex.quote(script)
                self.log(target, "BAG", "SSH를 통해 General 진단 rosbag 60초 시작")
                result = subprocess.run(base + [command], capture_output=True, timeout=95)
                if result.returncode not in (0, 124):
                    raise RuntimeError(result.stderr.decode(errors="replace")[-2000:])
                path = self.folder(target) / "diagnostics" / (name + ".tar.gz")
                path.parent.mkdir(parents=True, exist_ok=True)
                temporary = path.with_suffix(".part")
                with temporary.open("wb") as output:
                    result = subprocess.run(base + [f"docker exec ros2_jazzy_lidar tar -C {root}/{name} -czf - ."],
                                            stdout=output, stderr=subprocess.PIPE, timeout=90)
                if result.returncode:
                    raise RuntimeError(result.stderr.decode(errors="replace")[-2000:])
                with tarfile.open(temporary) as archive:
                    metadata = archive.extractfile("./metadata.yaml").read().decode()
                    count = re.search(r"\bmessage_count:\s*(\d+)", metadata)
                    if not count or int(count.group(1)) == 0 or not any(n.endswith(".mcap") for n in archive.getnames()):
                        raise RuntimeError("진단 bag이 정상적으로 기록되지 않았습니다")
                temporary.replace(path)
            self.log(target, "SSH", "원격 자료 저장 완료", file=str(path))
        except Exception as exc:
            self.log(target, "ERROR", f"SSH 수집 실패: {exc}")
        finally:
            target.diagnostic_pending = False

    def collect_sessions(self, target):
        """SSH fallback for General's existing detection recorder, no new remote API."""
        try:
            if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.@:-]*", target.ssh):
                raise ValueError("SSH는 user@host 형식으로 설정하세요")
            base = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "--", target.ssh]
            root = "/root/ros2_ws/cx128s2_dataset"
            script = ("import json,pathlib; out=[]\n"
                      f"for p in pathlib.Path({root!r}).glob('recording_*/session.json'):\n"
                      " try:\n"
                      "  d=json.loads(p.read_text())\n"
                      "  if d.get('complete') is True or ('complete' not in d and d.get('event_end_stamp_ns',0)>0): out.append(p.parent.name)\n"
                      " except (OSError,ValueError): pass\n"
                      "print(json.dumps(sorted(out)))")
            result = subprocess.run(base + ["docker exec ros2_jazzy_lidar python3 -c " + shlex.quote(script)],
                                    capture_output=True, timeout=25, check=True)
            completed = json.loads(result.stdout)
            known = {Path(job.get("recorder_path") or "").name for job in target.captures.copy().values()}
            for name in completed:
                if self.stop_event.is_set():
                    break
                if not isinstance(name, str) or not re.fullmatch(r"recording_[A-Za-z0-9_]+", name) or name in known:
                    continue
                capture_id = "ssh_" + name
                folder = self.folder(target) / "captures" / capture_id
                if (folder / "download.json").is_file():
                    continue
                folder.mkdir(parents=True, exist_ok=True)
                if shutil.disk_usage(folder).free < 1024 ** 3:
                    raise RuntimeError("General SSH 수집용 디스크 공간이 부족합니다")
                temporary = folder / "original.tar.gz.part"
                self.log(target, "DOWNLOAD", "General 감지 녹화 세션 SSH 회수", capture_id=capture_id)
                with temporary.open("wb") as output:
                    result = subprocess.run(base + [f"docker exec ros2_jazzy_lidar tar -C {root}/{name} -czf - ."],
                                            stdout=output, stderr=subprocess.PIPE, timeout=90)
                if result.returncode:
                    raise RuntimeError(result.stderr.decode(errors="replace")[-2000:])
                self.inspect_archive(temporary, folder, target)
                destination = folder / "original.tar.gz"
                temporary.replace(destination)
                job = {"capture_id": capture_id, "kind": "ssh_session", "state": "complete",
                       "recorder_path": root + "/" + name, "size_bytes": destination.stat().st_size}
                atomic_json(folder / "download.json", {"time": now_text(), "url": target.url,
                                                       "capture_id": capture_id, "job": job, "size_bytes": job["size_bytes"]})
                target.captures[capture_id] = job
                self.log(target, "DOWNLOAD", "General 감지 녹화 세션 수집 완료", file=str(destination))
        except Exception as exc:
            self.log(target, "ERROR", f"General 세션 SSH 수집 실패: {exc}")
        finally:
            target.sessions_pending = False

    def worker(self, target):
        while not self.stop_event.is_set():
            with self.lock:
                target.busy = True
            try:
                if self.demo:
                    self.demo_status(target)
                else:
                    self.poll_target(target)
                while not target.actions.empty():
                    action, value, submitted_at = target.actions.get_nowait()
                    if time.monotonic() - submitted_at > 30:
                        self.log(target, "ERROR", "오래 대기한 명령을 취소했습니다", action=action)
                        continue
                    if self.demo:
                        self.log(target, "DEMO", f"모의 명령: {action} {value}")
                    else:
                        self.action(target, action, value)
            except Exception as exc:
                target.error = str(exc)
                target.phase = "오류 / 재시도 대기"
                if time.monotonic() - target.checked_at > max(30, self.poll * 3):
                    target.connected = False
                    target.operated_since = None
                self.log(target, "ERROR", str(exc))
                # A failed command is consumed; ambiguous capture intents remain checkpointed.
            finally:
                with self.lock:
                    target.busy = False
            self.stop_event.wait(self.poll)

    def start(self):
        if self.started:
            return
        self.started = True
        for target in self.targets.values():
            thread = threading.Thread(target=self.worker, args=(target,), daemon=True)
            self.threads = [worker for worker in self.threads if worker.is_alive()]
            self.threads.append(thread)
            thread.start()

    def close(self):
        self.stop_event.set()
        if self.ros_context:
            self.ros_context.try_shutdown()

    def demo_status(self, target):
        target.connected, target.checked_at = True, time.monotonic()
        target.phase = "데모 · 실제 요청 없음"
        target.status = {"system_mode": target.name, "bridge_connected": True,
                         "lidar": {"packet_active": True, "pointcloud_active": True, "perception_active": True,
                                   "container_running": True, "lidar_bringup_running": True, "last_pointcloud_age_ms": 35},
                         "perception": ({"vehicle_exist": 1, "vehicle_status": "stopping", "vehicle_distance": 12.4,
                                         "vehicle_length": 4.8, "vehicle_count": 2} if target.name == "general" else
                                        {"state": "LIFTING", "beam_length": 5.6, "working_status": 1, "crane_moving": 0})}
        self.track_source = "데모 데이터 — 실제 ROS 관측 아님"
        self.track_received = time.monotonic()
        self.tracks["general:/tracked_objects"] = [
            {"id": 24, "class_name": "VEHICLE", "lifecycle_state": "TRACKED", "motion_state": "MOVING",
             "vehicle_status": "stopping", "distance": 12.4, "speed": 0.6, "lost_frames": 0,
             "decision_reason": "VEHICLE_SIZE_MATCH+CONFIRMED_BY_PERSISTENCE", "rejection_reason": ""},
            {"id": 31, "class_name": "VEHICLE", "lifecycle_state": "TRACKED_PREDICTED", "motion_state": "MOVING",
             "distance": 26.1, "speed": 1.2, "lost_frames": 2, "decision_reason": "BOUNDED_PREDICTION", "rejection_reason": ""}]

    def start_ros(self, target_name, prefix=""):
        def run():
            try:
                import rclpy
                from rclpy.context import Context
                from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
                from rclpy.qos import qos_profile_sensor_data
                from rcl_interfaces.msg import Log
                from std_msgs.msg import String
                from cx128s2_bev_perception.msg import TrackedObjectArray, CraneBeamStatus
                from rosidl_runtime_py.convert import message_to_ordereddict
                context = Context()
                self.ros_context = context
                rclpy.init(context=context)
                node = rclpy.create_node("external_lidar_control_observer", context=context)
                executor = SingleThreadedExecutor(context=context)
                executor.add_node(node)
                subscriptions = []
                saved_at = {}
                def receive(message, topic):
                    objects = [dict(message_to_ordereddict(obj)) for obj in message.objects]
                    with self.lock:
                        self.tracks[target_name + ":" + topic] = objects
                        self.ros_details[target_name + ":" + topic] = {"header": dict(message_to_ordereddict(message.header)), "objects": objects}
                        self.track_times[target_name + ":" + topic] = time.monotonic()
                        self.track_received = time.monotonic()
                        self.track_source = f"ROS {target_name} · Domain {os.environ.get('ROS_DOMAIN_ID', '0')} · {prefix or '/'}"
                    if time.monotonic() - saved_at.get(topic, 0) >= 1:
                        self.log(self.targets[target_name], "TRACKS", topic, objects=objects, header=dict(message_to_ordereddict(message.header)))
                        saved_at[topic] = time.monotonic()
                for topic in ("/detected_objects", "/tracked_objects", "/object_motion_states", "/road_vehicle_tracks", "/vehicle_identity_memory"):
                    subscriptions.append(node.create_subscription(TrackedObjectArray, prefix.rstrip("/") + topic,
                                                                  lambda msg, t=topic: receive(msg, t), qos_profile_sensor_data))
                for topic in ("/debug/track_table", "/debug/perception_diagnostics_table"):
                    subscriptions.append(node.create_subscription(String, prefix.rstrip("/") + topic,
                                                                  lambda msg, t=topic: self.log(self.targets[target_name], "ROS_DEBUG", t, text=msg.data), qos_profile_sensor_data))
                if target_name == "crane":
                    def crane_status(message):
                        value = dict(message_to_ordereddict(message))
                        with self.lock:
                            self.ros_details["crane:/crane_beam/status"] = value
                        if time.monotonic() - saved_at.get("crane_status", 0) >= 1:
                            self.log(self.targets[target_name], "ROS_STATE", "/crane_beam/status", value=value)
                            saved_at["crane_status"] = time.monotonic()
                    subscriptions.append(node.create_subscription(CraneBeamStatus, prefix.rstrip("/") + "/crane_beam/status", crane_status, qos_profile_sensor_data))
                    for topic in ("/crane_beam/diagnostics", "/crane_beam/command_status"):
                        subscriptions.append(node.create_subscription(String, prefix.rstrip("/") + topic,
                            lambda msg, t=topic: self.log(self.targets[target_name], "ROS_DEBUG", t, text=msg.data), qos_profile_sensor_data))
                subscriptions.append(node.create_subscription(Log, "/rosout", lambda msg: self.log(self.targets[target_name],
                                      "ROS_LOG", msg.msg, node=msg.name, level=msg.level), qos_profile_sensor_data))
                self.ros_status = "ROS 구독 중 · 대상 Domain/네임스페이스가 맞아야 합니다"
                try:
                    while not self.stop_event.is_set() and context.ok():
                        executor.spin_once(timeout_sec=.2)
                except ExternalShutdownException:
                    pass
                finally:
                    executor.remove_node(node)
                    executor.shutdown(timeout_sec=1)
                    node.destroy_node()
                    context.try_shutdown()
            except Exception as exc:
                self.ros_status = f"ROS 연결 불가: {exc}"
                self.log(self.targets[target_name], "ERROR", self.ros_status)
        thread = threading.Thread(target=run, daemon=True)
        self.threads = [worker for worker in self.threads if worker.is_alive()]
        self.threads.append(thread)
        thread.start()


class Desktop:
    def __init__(self, root, controller):
        import tkinter as tk
        from tkinter import ttk, messagebox, simpledialog, filedialog
        self.tk, self.ttk = tk, ttk
        self.messagebox, self.simpledialog, self.filedialog = messagebox, simpledialog, filedialog
        self.root, self.client = root, controller
        root.title("Guardian LiDAR · 외부 제어 / 수집 센터" + (" [DEMO]" if controller.demo else ""))
        root.geometry("1320x880")
        root.minsize(1080, 700)
        root.configure(bg="#edf2f7")
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TCheckbutton", background="#edf2f7", font=("Malgun Gothic", 10))
        style.configure("TLabelframe", background="#ffffff", borderwidth=1, relief="solid")
        style.configure("TLabelframe.Label", background="#edf2f7", font=("Malgun Gothic", 12, "bold"), foreground="#102a43")
        style.configure("Card.TFrame", background="#ffffff")
        style.configure("Card.TLabel", background="#ffffff", font=("Malgun Gothic", 10))
        style.configure("TFrame", background="#edf2f7")
        style.configure("TLabel", background="#edf2f7", font=("Malgun Gothic", 10))
        style.configure("Title.TLabel", font=("Malgun Gothic", 21, "bold"), foreground="#102a43")
        style.configure("Header.TLabel", font=("Malgun Gothic", 14, "bold"), foreground="#102a43")
        style.configure("TButton", padding=(12, 8), font=("Malgun Gothic", 10))
        style.configure("Treeview", rowheight=29, font=("Malgun Gothic", 10))
        style.configure("Treeview.Heading", font=("Malgun Gothic", 10, "bold"))
        top = ttk.Frame(root, padding=(22, 16))
        top.pack(fill="x")
        ttk.Label(top, text="LiDAR Control Center" + (" · DEMO" if controller.demo else ""), style="Title.TLabel").pack(side="left")
        ttk.Button(top, text="저장 폴더 열기", command=self.open_folder).pack(side="right")
        ttk.Button(top, text="접속 / SSH 설정", command=self.settings).pack(side="right", padx=8)
        self.notice = tk.StringVar(value="모니터링 모드 · 녹화/제어는 버튼을 눌러 실행합니다")
        self.notice_label = ttk.Label(root, textvariable=self.notice, padding=(22, 4), foreground="#526777")
        self.notice_label.pack(fill="x")
        cards = ttk.Frame(root, padding=(22, 10))
        cards.pack(fill="x")
        self.card_vars = {}
        for index, target in enumerate(controller.targets.values()):
            frame = ttk.LabelFrame(cards, text=target.name.upper(), padding=14)
            frame.grid(row=0, column=index, sticky="nsew", padx=(0, 12) if index == 0 else (0, 0))
            cards.columnconfigure(index, weight=1)
            variables = {key: tk.StringVar() for key in ("url", "health", "perception", "phase", "error")}
            self.card_vars[target.name] = variables
            for key in variables:
                ttk.Label(frame, textvariable=variables[key], wraplength=585, style="Card.TLabel",
                          foreground="#b42318" if key == "error" else "#243b53").pack(anchor="w", pady=3)
            buttons = ttk.Frame(frame, style="Card.TFrame")
            buttons.pack(fill="x", pady=(10, 0))
            ttk.Button(buttons, text="정상 모드", command=lambda n=target.name: self.command(n, "normal")).pack(side="left")
            ttk.Button(buttons, text="대기 모드", command=lambda n=target.name: self.command(n, "standby")).pack(side="left", padx=5)
            ttk.Button(buttons, text="10초 녹화" if target.name == "general" else "60초 녹화",
                       command=lambda n=target.name: self.client.submit(n, "capture", 10 if n == "general" else 60)).pack(side="left")
            ttk.Button(buttons, text="작업 ID 조회", command=lambda n=target.name: self.recover(n)).pack(side="left", padx=5)
            options = ttk.Frame(frame, style="Card.TFrame")
            options.pack(fill="x", pady=(8, 0))
            collect = tk.BooleanVar(value=target.collect)
            ttk.Checkbutton(options, text="완료 파일 자동 다운로드", variable=collect,
                            command=lambda n=target.name, v=collect: self.client.submit(n, "collect", v.get())).pack(side="left")
            if target.name == "crane":
                repeat = tk.BooleanVar(value=target.repeat)
                ttk.Checkbutton(options, text="60초 요청 → 수집 반복", variable=repeat,
                                command=lambda v=repeat: self.client.submit("crane", "repeat", v.get())).pack(side="left", padx=10)
            if target.name == "general":
                diagnostic = tk.BooleanVar(value=target.auto_diagnostic)
                ttk.Checkbutton(options, text="3시간 진단 자동 (SSH)", variable=diagnostic,
                                command=lambda v=diagnostic: self.client.submit("general", "auto_diagnostic", v.get())).pack(side="left", padx=10)
            read_buttons = ttk.Frame(frame, style="Card.TFrame")
            read_buttons.pack(fill="x", pady=(8, 0))
            ttk.Button(read_buttons, text="서버 로그 가져오기 (SSH)", command=lambda n=target.name: self.client.submit(n, "journal")).pack(side="left")
            ttk.Button(read_buttons, text="추적 스냅샷 (SSH)", command=lambda n=target.name: self.client.submit(n, "tracks")).pack(side="left", padx=6)
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True, padx=22, pady=10)
        track_page = ttk.Frame(notebook, padding=10)
        notebook.add(track_page, text="실시간 추적 / 예측 / 이유")
        self.track_notice = tk.StringVar()
        ttk.Label(track_page, textvariable=self.track_notice).pack(anchor="w", pady=(0, 8))
        columns = ("source", "id", "class", "lifecycle", "predicted", "motion", "distance", "speed", "lost", "reason", "reject")
        self.track_tree = self.tree(track_page, columns, ("출처", "ID", "분류", "수명 상태", "예측", "이동", "거리 m", "속도 m/s", "미관측", "판정 이유", "거부 이유"))
        self.track_tree.column("source", width=220)
        self.track_tree.column("id", width=55)
        self.track_tree.column("lifecycle", width=160)
        self.track_tree.column("reason", width=330)
        self.track_tree.column("reject", width=230)
        self.visible_tracks = {}
        self.track_tree.bind("<Double-1>", self.track_details)
        self.track_tree.tag_configure("predicted", background="#fff4d6")
        self.track_tree.tag_configure("observed", background="#e5f5ef")
        self.track_tree.tag_configure("stale", background="#edf0f3", foreground="#64748b")
        files_page = ttk.Frame(notebook, padding=10)
        notebook.add(files_page, text="녹화 / 수집 파일")
        ttk.Label(files_page, text="original.tar.gz: 받은 원본 · Crane raw_1hz.tar.gz: 초당 1개 보관본 · summary/frames/objects: 분류된 정보").pack(anchor="w", pady=(0, 8))
        self.file_tree = self.tree(files_page, ("target", "id", "state", "size", "path"), ("대상", "작업 ID", "상태", "크기", "저장 위치"))
        self.file_tree.column("path", width=650)
        ttk.Button(files_page, text="선택한 파일의 객체 정보 보기", command=self.show_objects).pack(anchor="e", pady=8)
        logs_page = ttk.Frame(notebook, padding=10)
        notebook.add(logs_page, text="통신 / 명령 / 알림 로그")
        filter_row = ttk.Frame(logs_page)
        filter_row.pack(fill="x")
        self.log_filter = tk.StringVar(value="전체")
        ttk.Combobox(filter_row, textvariable=self.log_filter, values=("전체", "ERROR", "CAPTURE", "DOWNLOAD", "COMMAND", "ALERT", "HTTP", "CAPABILITY", "TRACKS", "ROS_STATE", "ROS_LOG", "ROS_DEBUG", "SSH", "BAG", "DEMO"), state="readonly", width=18).pack(side="left")
        ttk.Button(filter_row, text="로그 JSON 내보내기", command=self.export_logs).pack(side="right")
        self.log_tree = self.tree(logs_page, ("time", "target", "category", "message"), ("시각", "대상", "분류", "내용"))
        self.log_tree.column("message", width=750)
        self.visible_logs = {}
        self.log_tree.bind("<Double-1>", self.log_details)
        self.log_tree.tag_configure("ERROR", foreground="#b42318")
        raw_page = ttk.Frame(notebook, padding=10)
        notebook.add(raw_page, text="API / ROS 원문 · 지원 범위")
        self.raw_text = tk.Text(raw_page, font=("DejaVu Sans Mono", 10), wrap="none", bg="#f8fafc", relief="flat", height=10)
        self.raw_text.pack(fill="both", expand=True)
        footer = ttk.Frame(root, padding=(22, 8))
        footer.pack(fill="x")
        self.footer = tk.StringVar()
        ttk.Label(footer, textvariable=self.footer, wraplength=850).pack(side="left")
        ttk.Button(footer, text="General 진단 rosbag 60초 (SSH)", command=lambda: self.command("general", "bag")).pack(side="right")
        root.protocol("WM_DELETE_WINDOW", self.close)
        controller.start()
        self.refresh()

    def tree(self, parent, columns, headings):
        frame = self.ttk.Frame(parent)
        frame.pack(fill="both", expand=True)
        tree = self.ttk.Treeview(frame, columns=columns, show="headings", height=5)
        for column, heading in zip(columns, headings):
            tree.heading(column, text=heading)
            tree.column(column, width=100, minwidth=60)
        vertical = self.ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
        horizontal = self.ttk.Scrollbar(frame, orient="horizontal", command=tree.xview)
        tree.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        tree.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        return tree

    def command(self, name, action):
        if self.messagebox.askyesno("LiDAR 명령", f"{name.upper()}에 {action} 명령을 보낼까요?"):
            self.client.submit(name, action)

    def recover(self, name):
        value = self.simpledialog.askstring("기존 작업 조회 / 복구", "서버에서 반환된 capture_id를 입력하세요. 새 녹화는 시작하지 않습니다.")
        if value:
            self.client.submit(name, "recover", value.strip())

    def settings(self):
        window = self.tk.Toplevel(self.root)
        window.title("접속 설정")
        values = {}
        for row, target in enumerate(self.client.targets.values()):
            self.ttk.Label(window, text=target.name.upper() + " HTTP URL").grid(row=row * 2, column=0, padx=12, pady=8)
            url = self.tk.StringVar(value=target.url)
            self.ttk.Entry(window, textvariable=url, width=55).grid(row=row * 2, column=1, padx=12)
            self.ttk.Label(window, text="선택적 SSH user@host").grid(row=row * 2 + 1, column=0, padx=12, pady=8)
            ssh = self.tk.StringVar(value=target.ssh)
            self.ttk.Entry(window, textvariable=ssh, width=55).grid(row=row * 2 + 1, column=1, padx=12)
            values[target.name] = (url, ssh)
        def save():
            if any(t.busy or t.capture_id or t.uncertain or t.diagnostic_pending or t.sessions_pending for t in self.client.targets.values()):
                self.messagebox.showinfo("진행 중", "진행 중인 요청이 있거나 상태 조회 중입니다. 작업이 없는 시점에 변경하세요.")
                return
            for name, (url, ssh) in values.items():
                target = self.client.targets[name]
                target.url, target.ssh = url.get().strip().rstrip("/"), ssh.get().strip()
                target.list_supported, target.connected = None, False
            atomic_json(self.client.output / "connection.json", {name: {"url": t.url, "ssh": t.ssh} for name, t in self.client.targets.items()})
            window.destroy()
        self.ttk.Button(window, text="저장", command=save).grid(row=5, column=1, padx=12, pady=12)

    def open_folder(self):
        self.client.output.mkdir(parents=True, exist_ok=True)
        subprocess.Popen(["xdg-open", str(self.client.output)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def track_details(self, _event=None):
        selected = self.track_tree.selection()
        if selected and selected[0] in self.visible_tracks:
            self.text_window("추적 관측 상세 — 출처/수신 시각 확인", json.dumps(self.visible_tracks[selected[0]], ensure_ascii=False, indent=2))

    def log_details(self, _event=None):
        selected = self.log_tree.selection()
        if selected and selected[0] in self.visible_logs:
            self.text_window("통신 / 로그 상세 원문", json.dumps(self.visible_logs[selected[0]], ensure_ascii=False, indent=2))

    def export_logs(self):
        name = self.filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if name:
            with self.client.lock:
                atomic_json(Path(name), list(self.client.events))

    def show_objects(self):
        selection = self.file_tree.selection()
        if not selection:
            return
        values = self.file_tree.item(selection[0], "values")
        path = self.client.output / values[0] / "captures" / values[1] / "objects.json"
        self.text_window("저장된 객체 정보 · 실시간 관측 아님", path.read_text() if path.exists() else "이 파일에는 객체 정보가 없습니다.")

    def text_window(self, title, value):
        window = self.tk.Toplevel(self.root)
        window.title(title)
        window.geometry("1000x650")
        text = self.tk.Text(window, wrap="none", font=("DejaVu Sans Mono", 10))
        text.pack(fill="both", expand=True)
        text.insert("1.0", value)
        text.configure(state="disabled")

    def refresh(self):
        with self.client.lock:
            raw = {}
            for name, target in self.client.targets.items():
                variables = self.card_vars[name]
                fresh = target.connected and time.monotonic() - target.checked_at < max(30, self.client.poll * 3)
                lidar, perception = target.status.get("lidar", {}), target.status.get("perception", {})
                variables["url"].set(target.url + (f" · {time.monotonic() - target.checked_at:.0f}초 전 조회" if target.checked_at else ""))
                yes = lambda value: "정상" if fresh and value else "미확인"
                variables["health"].set(f"접속 {'정상' if fresh else '끊김'}  |  UDP {yes(lidar.get('packet_active'))}  |  PointCloud {yes(lidar.get('pointcloud_active'))}  |  인지 {yes(lidar.get('perception_active'))}  |  Docker {yes(lidar.get('container_running'))}")
                if name == "general":
                    variables["perception"].set(f"차량 {perception.get('vehicle_count', '—')}대  ·  상태 {perception.get('vehicle_status', '—')}  ·  거리 {perception.get('vehicle_distance', '—')}m")
                else:
                    variables["perception"].set(f"작업 {perception.get('state', '—')}  ·  빔 길이 {perception.get('beam_length', '—')}m  ·  이동 {perception.get('crane_moving', '—')}")
                variables["phase"].set(f"{target.phase}  |  ID {target.capture_id or '—'}")
                variables["error"].set(target.error or ("기존 General 수집기와 중복 방지 중" if name == "crane" and self.client.crane_coordinator_conflict() else ""))
                raw[name] = {"status": target.status, "capture_list_supported": target.list_supported,
                             "remote_journal_http": "미지원; SSH로 선택 수집", "recorder_ready_http": "구버전 미지원; 요청/파일 완료로 검증"}
            track_position = self.track_tree.yview()[0]
            track_selection = self.track_tree.selection()
            self.track_tree.delete(*self.track_tree.get_children())
            self.visible_tracks = {}
            age = time.monotonic() - self.client.track_received if self.client.track_received else None
            self.track_notice.set(self.client.track_source + (f" · 마지막 수신 {age:.1f}초 전" if age is not None else "") + " · HTTP 요약만으로 예측 여부를 추정하지 않습니다")
            for source, objects in self.client.tracks.items():
                for index, obj in enumerate(objects):
                    stale = not self.client.demo and time.monotonic() - self.client.track_times.get(source, 0) > 5
                    predicted = obj.get("lifecycle_state") == "TRACKED_PREDICTED"
                    lifecycle = obj.get("lifecycle_state", "미제공")
                    row_id = source + ":" + str(obj.get("id", index)) + ":" + str(index)
                    self.visible_tracks[row_id] = {"source": source, "stale": stale, "object": obj}
                    self.track_tree.insert("", "end", iid=row_id, values=(source, obj.get("id", "—"), obj.get("class_name", "—"), lifecycle,
                        "예측" if predicted else ("아니오" if lifecycle == "TRACKED" else "상태 참조"), obj.get("motion_state", "—"),
                        f"{obj.get('distance', 0):.2f}", f"{obj.get('speed', 0):.2f}", obj.get("lost_frames", "—"),
                        obj.get("decision_reason") or "미제공", obj.get("rejection_reason") or "—"), tags=("stale" if stale else ("predicted" if predicted else "observed"),))
            self.track_tree.selection_set([item for item in track_selection if self.track_tree.exists(item)])
            self.track_tree.yview_moveto(track_position)
            selected_files = self.file_tree.selection()
            self.file_tree.delete(*self.file_tree.get_children())
            for name, target in self.client.targets.items():
                for capture_id, job in list(target.captures.copy().items()):
                    folder = self.client.folder(target) / "captures" / capture_id
                    self.file_tree.insert("", "end", iid=name + ":" + capture_id, values=(name, capture_id, "수집 완료" if (folder / "download.json").exists() else job.get("state", "—"),
                        f"{(job.get('size_bytes') or 0) / 1024 ** 2:.1f} MiB", str(folder) if folder.exists() else "서버에 있음"))
            self.file_tree.selection_set([item for item in selected_files if self.file_tree.exists(item)])
            log_position = self.log_tree.yview()[0]
            selected_logs = self.log_tree.selection()
            self.log_tree.delete(*self.log_tree.get_children())
            self.visible_logs = {}
            for event in list(self.client.events)[-300:]:
                if self.log_filter.get() in ("전체", event["category"]):
                    row_id = "log_" + str(event["sequence"])
                    self.visible_logs[row_id] = event
                    self.log_tree.insert("", "end", iid=row_id, values=(event["time"], event["target"], event["category"], event["message"]), tags=(event["category"],))
            self.log_tree.selection_set([item for item in selected_logs if self.log_tree.exists(item)])
            self.log_tree.yview_moveto(log_position)
            if self.client.ros_details:
                raw["ROS 관측 원문"] = dict(self.client.ros_details)
            raw_position = self.raw_text.yview()[0]
            self.raw_text.configure(state="normal")
            self.raw_text.delete("1.0", "end")
            self.raw_text.insert("1.0", json.dumps(raw, ensure_ascii=False, indent=2))
            self.raw_text.configure(state="disabled")
            self.raw_text.yview_moveto(raw_position)
            general = self.client.targets["general"]
            elapsed = general.status.get("operation_elapsed_sec", time.monotonic() - general.operated_since if general.operated_since else 0)
            alarm = general.status.get("operation_alert") or ("3시간 경과" if general.alert_sent else "없음")
            if self.client.demo:
                self.notice.set("데모 화면 · 실제 통신 / 녹화 / 제어를 실행하지 않습니다")
            elif general.status.get("operation_alert") or general.alert_sent:
                self.notice.set("General 연속 차량 인지 3시간 알림 · 진단 상태와 로그를 확인하세요")
                self.notice_label.configure(foreground="#b45309")
            elif self.client.targets["crane"].repeat:
                conflict = self.client.crane_coordinator_conflict()
                self.notice.set(conflict or "외부 Crane 수집 반복 활성 · 반복을 끄면 현재 작업까지만 완료/회수합니다")
                self.notice_label.configure(foreground="#b45309" if conflict else "#166534")
            else:
                self.notice.set("모니터링 모드 · 녹화/제어는 버튼을 눌러 실행합니다")
                self.notice_label.configure(foreground="#526777")
            self.footer.set(f"General 차량 인지 {float(elapsed) / 3600:.2f}시간 · 알림 {alarm if isinstance(alarm, str) else alarm.get('message', '있음')}  |  {self.client.ros_status}")
        self.root.after(1000, self.refresh)

    def close(self):
        self.client.close()
        self.root.destroy()


class Terminal:
    """Standard-library curses dashboard using the same guarded controller."""
    def __init__(self, screen, controller):
        import curses
        self.curses, self.screen, self.client = curses, screen, controller
        self.target, self.page, self.selected = "general", 0, 0
        self.message, self.details, self.detail_offset = "", None, 0
        self.horizontal = 0
        self.ascii = False
        self.rows = []
        screen.keypad(True)
        screen.timeout(250)
        try:
            curses.curs_set(0)
            curses.start_color()
            curses.use_default_colors()
            for index, color in enumerate((curses.COLOR_CYAN, curses.COLOR_GREEN,
                                           curses.COLOR_YELLOW, curses.COLOR_RED), 1):
                curses.init_pair(index, color, -1)
        except curses.error:
            pass

    def write(self, row, text, style=0, column=0, limit=None):
        height, width = self.screen.getmaxyx()
        if not 0 <= row < height or not 0 <= column < width - 1:
            return
        available = min(width - column - 1, limit if limit is not None else width)
        # curses columns count CJK characters twice; also avoid the bottom-right cell.
        used, characters = 0, []
        for character in str(text).replace("\n", " ").replace("\t", " "):
            if unicodedata.category(character).startswith("C"):
                continue
            size = 0 if unicodedata.combining(character) else (2 if unicodedata.east_asian_width(character) in "WF" else 1)
            if used + size > available:
                break
            characters.append(character)
            used += size
        try:
            self.screen.addstr(row, column, "".join(characters), style)
        except self.curses.error:
            pass

    def prompt(self, label, default=""):
        """Unicode input without leaving curses; Escape cancels."""
        value = default
        self.screen.timeout(-1)
        try:
            while True:
                height, _ = self.screen.getmaxyx()
                self.screen.move(height - 1, 0)
                self.screen.clrtoeol()
                self.write(height - 1, f"{label}: {value}  [Enter 확인 / Esc 취소]", self.curses.A_REVERSE)
                self.screen.refresh()
                key = self.screen.get_wch()
                if key in ("\n", "\r", self.curses.KEY_ENTER):
                    return value.strip()
                if key == "\x1b":
                    return None
                if key in ("\b", "\x7f", self.curses.KEY_BACKSPACE):
                    value = value[:-1]
                elif key == "\x15":
                    value = ""
                elif isinstance(key, str) and key.isprintable():
                    value += key
        finally:
            self.screen.timeout(250)

    def command(self, action, value=None):
        target = self.client.targets[self.target]
        if action in ("normal", "standby", "capture", "bag"):
            answer = self.prompt(f"{self.target.upper()} {action} {value or ''} 실행? y 입력")
            if answer != "y":
                self.message = "명령 취소"
                return
        self.client.submit(self.target, action, value)
        self.message = f"{self.target}: {action} 요청 · 결과는 상태/로그에서 확인"

    def settings(self):
        target = self.client.targets[self.target]
        if any(t.busy or t.capture_id or t.uncertain or t.diagnostic_pending or t.sessions_pending
               for t in self.client.targets.values()):
            self.message = "요청/조회 진행 중 · 작업이 없는 시점에 설정하세요"
            return
        url = self.prompt(f"{self.target} HTTP URL (Ctrl-U 전체 삭제)", target.url)
        if url is None:
            return
        parsed = urllib.parse.urlparse(url)
        if parsed.scheme not in ("http", "https") or not parsed.netloc or parsed.query or parsed.fragment:
            self.message = "http(s)://주소:포트 형태로 입력하세요"
            return
        ssh = self.prompt("선택적 SSH user@host (비워두면 사용 안 함)", target.ssh)
        if ssh is None:
            return
        with self.client.lock:
            if any(t.busy or t.capture_id or t.uncertain or t.diagnostic_pending or t.sessions_pending
                   for t in self.client.targets.values()):
                self.message = "설정 입력 중 요청이 시작됐습니다 · 변경 보류"
                return
            target.url, target.ssh = url.rstrip("/"), ssh
            target.connected, target.checked_at, target.status = False, 0, {}
            target.list_supported = None
            target.operated_since, target.alert_sent = None, False
            target.repeat = False
            atomic_json(self.client.output / "connection.json", {
                name: {"url": item.url, "ssh": item.ssh} for name, item in self.client.targets.items()})
        self.message = "접속 설정 저장 · 다음 조회에서 확인"

    def render(self):
        self.screen.erase()
        curses = self.curses
        height, width = self.screen.getmaxyx()
        self.write(0, ("[L]" if self.ascii else "◈") + "  GUARDIAN LiDAR  /  CONTROL CENTER" + ("  [DEMO · 실제 요청 없음]" if self.client.demo else ""), curses.A_BOLD | curses.color_pair(1))
        if height < 18 or width < 70:
            self.write(2, "터미널을 70열 × 18행 이상으로 늘려주세요. q: 종료")
            self.screen.refresh()
            return
        if self.details is not None:
            self.write(1, "상세 · ↑↓/PgUp/PgDn 세로 · ←→ 가로 · Esc/Enter 닫기", curses.A_REVERSE)
            lines = self.details.splitlines()
            self.detail_offset = min(self.detail_offset, max(0, len(lines) - (height - 3)))
            for row, line in enumerate(lines[self.detail_offset:self.detail_offset + height - 3], 2):
                self.write(row, line[self.horizontal:])
            self.screen.refresh()
            return
        with self.client.lock:
            wide = width >= 110
            card_width = (width - 3) // 2 if wide else width - 1
            card_height = 7 if wide else 5
            for index, target in enumerate(self.client.targets.values()):
                base = 2 if wide else 1 + index * card_height
                column = index * (card_width + 2) if wide else 0
                border_style = curses.color_pair(1) | (curses.A_BOLD if target.name == self.target else curses.A_DIM)
                horizontal, vertical = ('-', '|') if self.ascii else ('─', '│')
                corners = ('+', '+', '+', '+') if self.ascii else ('┌', '┐', '└', '┘')
                self.write(base, corners[0] + horizontal * (card_width - 2) + corners[1], border_style, column, card_width)
                self.write(base + card_height - 1, corners[2] + horizontal * (card_width - 2) + corners[3], border_style, column, card_width)
                for row in range(base + 1, base + card_height - 1):
                    self.write(row, vertical, border_style, column, 1)
                    self.write(row, vertical, border_style, column + card_width - 1, 1)
                fresh = self.client.ready(target)
                lidar, perception = target.status.get("lidar", {}), target.status.get("perception", {})
                flag = lambda key: ('+' if self.ascii else '●') if fresh and lidar.get(key) else ('?' if self.ascii else '○')
                marker = ('>' if self.ascii else '▶') if target.name == self.target else " "
                age = f"{time.monotonic() - target.checked_at:.0f}s" if target.checked_at else "--"
                title = f" {marker} {target.name.upper()}  {'ONLINE' if fresh else 'OFFLINE'}  {age} "
                self.write(base, title, curses.A_BOLD | curses.color_pair(2 if fresh else 4), column + 2, card_width - 4)
                self.write(base + 1, f"UDP {flag('packet_active')}  PCD {flag('pointcloud_active')}  인지 {flag('perception_active')}  Docker {flag('container_running')}", 0, column + 2, card_width - 4)
                summary = (f"차량 {perception.get('vehicle_count', '?')}대  {perception.get('vehicle_status', '?')}  {perception.get('vehicle_distance', '?')}m" if target.name == "general" else
                           f"작업 {perception.get('state', '?')}  빔 {perception.get('beam_length', '?')}m")
                self.write(base + 2, summary, curses.A_BOLD, column + 2, card_width - 4)
                self.write(base + 3, f"수집 {'ON' if target.collect else 'OFF'}  반복 {'ON' if target.repeat else 'OFF'}  {target.error or target.phase}", curses.color_pair(4 if target.error else 1), column + 2, card_width - 4)
                if wide:
                    self.write(base + 4, target.url, curses.A_DIM, column + 2, card_width - 4)
                    self.write(base + 5, f"ID {target.capture_id or '-'}" + (' · 결과 불명' if target.uncertain else ''), 0, column + 2, card_width - 4)
            tab_row = 10 if wide else 11
            pages = ("1 상태", "2 추적", "3 파일", "4 로그", "5 원문")
            self.write(tab_row, "  ".join(f"[ {label} ]" if i == self.page else f"  {label}  " for i, label in enumerate(pages)), curses.A_REVERSE | curses.color_pair(1))
            self.rows = []
            if self.page == 0:
                target = self.client.targets[self.target]
                general = self.client.targets['general']
                elapsed = general.status.get('operation_elapsed_sec', time.monotonic() - general.operated_since if general.operated_since else 0)
                filled = min(18, max(0, int(elapsed / 10800 * 18)))
                bar = ('#' if self.ascii else '━') * filled + ('-' if self.ascii else '·') * (18 - filled)
                lines = [f"연속 차량 인지  [{bar}]  {elapsed / 3600:.2f} / 3.00 h",
                         f"선택 {self.target.upper()}  ·  SSH 자동 진단 {'ON' if general.auto_diagnostic else 'OFF'}  ·  감지 중단 시 초기화",
                         f"HTTP {target.url} · 현재 작업 ID {target.capture_id or '-'}",
                         f"저장 위치: {self.client.output}", f"SSH: {target.ssh or '미설정'} · {self.client.ros_status}",
                         self.client.crane_coordinator_conflict() or "Crane 기존 수집기 중복 없음",
                         "HTTP에는 추적 ID/예측/판단 이유가 없습니다. ROS 구독 또는 SSH 스냅샷을 사용하세요.",
                         "n 정상 / s 대기 / r 녹화 / a 완료 파일 자동 수집 / b Crane 반복",
                         "l SSH 로그 / t SSH 추적 / d General 60s rosbag / x 3시간 SSH 진단",
                         "i 작업 ID 조회 / e HTTP·SSH 설정 / Enter 상세 / q 종료"]
                self.rows = [(line, {'target': self.target, 'status': target.status}, 0) for line in lines]
            elif self.page == 1:
                self.write(tab_row + 1, self.client.track_source + " · 흐림=오래됨 / 노랑=예측", curses.A_DIM)
                for source, objects in list(self.client.tracks.items()):
                    stale = time.monotonic() - self.client.track_times.get(source, self.client.track_received) > 5
                    for obj in list(objects):
                        state = obj.get('lifecycle_state', '?')
                        color = curses.A_DIM if stale else curses.color_pair(3 if state == 'TRACKED_PREDICTED' else 2)
                        icon = ('~' if self.ascii else '◇') if state == 'TRACKED_PREDICTED' else ('+' if self.ascii else '●')
                        line = f"{icon} {source.split(':')[0]:7} #{str(obj.get('id', '?')):4} {obj.get('class_name', '?'):8} {state:18} {obj.get('distance', '?')}m  미관측 {obj.get('lost_frames', '?')}  {obj.get('decision_reason', '')}"
                        self.rows.append((line, {'source': source, 'stale': stale, **obj}, color))
            elif self.page == 2:
                for target in self.client.targets.values():
                    for capture_id, job in list(target.captures.items()):
                        folder = self.client.folder(target) / 'captures' / capture_id
                        line = f"{target.name} {capture_id} {job.get('state', '?')} {job.get('size_bytes') or 0:,}B {'LOCAL' if (folder / 'original.tar.gz').exists() else 'REMOTE'}"
                        self.rows.append((line, {**job, 'local_folder': str(folder)}, 0))
            elif self.page == 3:
                for event in reversed(list(self.client.events)):
                    color = curses.color_pair(4 if event['category'] == 'ERROR' else 3 if event['category'] == 'ALERT' else 1)
                    self.rows.append((f"{event['time'][11:19]} {event['target']} [{event['category']}] {event['message']}", event, color))
            else:
                raw = {'status': self.client.targets[self.target].status, 'ROS': dict(self.client.ros_details)}
                self.rows = [(line, raw, 0) for line in json.dumps(raw, ensure_ascii=False, indent=2).splitlines()]
            self.selected = min(self.selected, max(0, len(self.rows) - 1))
            body_row = tab_row + 2
            available = max(1, height - body_row - 3)
            start = max(0, self.selected - available + 1)
            for index, (line, _, style) in enumerate(self.rows[start:start + available], start):
                self.write(body_row + index - start, (("> " if self.ascii else "▸ ") if index == self.selected else "  ") + line[self.horizontal:],
                           style | (curses.A_REVERSE if index == self.selected else 0))
            if not self.rows:
                self.write(body_row, "  기록된 정보가 없습니다 · 연결/ROS/SSH 상태를 확인하세요", curses.A_DIM)
        self.write(height - 3, self.message or ("+ 정상 / ? 미확인  ·  u 기호 전환" if self.ascii else "● 정상  ○ 미확인  ◇ 예측  ·  u 기호 전환"), curses.color_pair(3))
        self.write(height - 2, "Tab 대상  1-5 화면  ↑↓←→ 이동  Enter 상세  n/s/r 제어  ? 도움  q 종료", curses.A_REVERSE)
        self.write(height - 1, "종료 시 새 반복 요청 중지 · 이미 시작한 원격 녹화는 서버에서 완료 · 원본/진행 ID 보존")
        self.screen.refresh()

    def key(self, key):
        curses = self.curses
        if self.details is not None:
            if key in ('\x1b', '\n', '\r', curses.KEY_ENTER):
                self.details = None
            elif key in (curses.KEY_DOWN, curses.KEY_NPAGE):
                self.detail_offset += 1 if key == curses.KEY_DOWN else 10
            elif key in (curses.KEY_UP, curses.KEY_PPAGE):
                self.detail_offset = max(0, self.detail_offset - (1 if key == curses.KEY_UP else 10))
            elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
                self.horizontal = max(0, self.horizontal + (10 if key == curses.KEY_RIGHT else -10))
            return True
        if key in ('q', 'Q'):
            return False
        if key == 'u':
            self.ascii = not self.ascii
            return True
        if key in ('g', 'c', '\t'):
            self.target = ('general' if self.target == 'crane' else 'crane') if key == '\t' else ('general' if key == 'g' else 'crane')
            self.selected = 0
        elif isinstance(key, str) and key in '12345':
            self.page, self.selected, self.horizontal = int(key) - 1, 0, 0
        elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
            self.horizontal = max(0, self.horizontal + (10 if key == curses.KEY_RIGHT else -10))
        elif key in (curses.KEY_UP, curses.KEY_PPAGE):
            self.selected = max(0, self.selected - (1 if key == curses.KEY_UP else 10))
        elif key in (curses.KEY_DOWN, curses.KEY_NPAGE):
            self.selected = min(max(0, len(self.rows) - 1), self.selected + (1 if key == curses.KEY_DOWN else 10))
        elif key in ('\n', '\r', curses.KEY_ENTER) and self.rows:
            self.details = json.dumps(self.rows[self.selected][1], ensure_ascii=False, indent=2)
            self.detail_offset, self.horizontal = 0, 0
        elif key == '?':
            self.page, self.selected = 0, 0
        elif key in ('n', 's', 'r', 'l', 't', 'd'):
            if key == 'd' and self.target != 'general':
                self.message = '진단 rosbag은 General에서만 지원합니다'
            else:
                self.command({'n': 'normal', 's': 'standby', 'r': 'capture', 'l': 'journal', 't': 'tracks', 'd': 'bag'}[key],
                             (10 if self.target == 'general' else 60) if key == 'r' else None)
        elif key in ('a', 'b', 'x'):
            name, action = ('crane', 'repeat') if key == 'b' else ('general', 'auto_diagnostic') if key == 'x' else (self.target, 'collect')
            target = self.client.targets[name]
            value = not getattr(target, action)
            if value and key in ('b', 'x') and self.prompt(f'{name} {action} 자동 실행? y 입력') != 'y':
                self.message = '자동 실행 취소'
            else:
                self.client.submit(name, action, value)
                self.message = f'{name} {action}: {"ON" if value else "OFF"}'
        elif key == 'i':
            value = self.prompt('서버 capture_id')
            if value:
                self.command('recover', value)
        elif key == 'e':
            self.settings()
        return True

    def run(self):
        self.client.start()
        try:
            while not self.client.stop_event.is_set():
                self.render()
                try:
                    key = self.screen.get_wch()
                except self.curses.error:
                    continue
                if not self.key(key):
                    break
        finally:
            self.client.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--general", default="http://192.168.223.152:8765")
    parser.add_argument("--crane", default="http://192.168.223.151:8765")
    parser.add_argument("--output", default=str(OUTPUT_DIRECTORY),
                        help="local save directory (default: lidar_collections beside this script)")
    parser.add_argument("--poll", type=float, default=5.0)
    parser.add_argument("--general-ssh", default="")
    parser.add_argument("--crane-ssh", default="")
    parser.add_argument("--ros-target", choices=("general", "crane"))
    parser.add_argument("--ros-prefix", default="")
    parser.add_argument("--demo", action="store_true", help="no real network or commands")
    display_mode = parser.add_mutually_exclusive_group()
    display_mode.add_argument("--headless", action="store_true", help="background polling without interactive UI")
    display_mode.add_argument("--tui", action="store_true", help="keyboard terminal dashboard (no Tkinter)")
    parser.add_argument("--collect-general", action="store_true")
    parser.add_argument("--auto-diagnostic", action="store_true", help="opt-in General three-hour SSH bag when the server does not own it")
    parser.add_argument("--auto-crane", action="store_true", help="opt-in; existing coordinator guard still applies")
    args = parser.parse_args()
    if args.poll < 1:
        parser.error("--poll must be at least 1 second")
    try:
        for name in ("general", "crane"):
            (Path(args.output).expanduser() / name).mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        parser.error(f"저장 폴더를 만들 수 없습니다: {exc}. 쓰기 가능한 --output 경로를 지정하세요.")
    settings = Path(args.output).expanduser() / "connection.json"
    if settings.exists():
        saved = json.loads(settings.read_text())
        for name in ("general", "crane"):
            if not any(arg == "--" + name or arg.startswith("--" + name + "=") for arg in os.sys.argv):
                setattr(args, name, saved.get(name, {}).get("url", getattr(args, name)))
            if not any(arg == "--" + name + "-ssh" or arg.startswith("--" + name + "-ssh=") for arg in os.sys.argv):
                setattr(args, name + "_ssh", saved.get(name, {}).get("ssh", getattr(args, name + "_ssh")))
    client = ControlCenter(args.general, args.crane, args.output, args.poll, args.demo)
    for name in ("general", "crane"):
        client.targets[name].ssh = getattr(args, name + "_ssh")
    if args.ros_target and not args.demo:
        client.start_ros(args.ros_target, args.ros_prefix)
    if args.collect_general:
        client.submit("general", "collect", True)
    if args.auto_diagnostic:
        client.submit("general", "auto_diagnostic", True)
    if args.auto_crane:
        client.submit("crane", "repeat", True)
    if args.headless:
        client.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            client.close()
    else:
        root = None
        try:
            if not args.tui:
                import tkinter as tk
                root = tk.Tk()
        except Exception as exc:
            print(f"GUI 실행 불가: {exc}. 터미널 UI로 전환합니다.", file=os.sys.stderr)
        if root is not None:
            Desktop(root, client)
            root.mainloop()
        else:
            if not os.sys.stdin.isatty() or not os.sys.stdout.isatty():
                client.close()
                parser.exit(1, "터미널 UI는 대화형 TTY가 필요합니다. 터미널/SSH -t에서 실행하거나 --headless를 사용하세요.\n")
            try:
                import curses
                curses.wrapper(lambda screen: Terminal(screen, client).run())
            except KeyboardInterrupt:
                pass
            finally:
                client.close()


if __name__ == "__main__":
    main()
