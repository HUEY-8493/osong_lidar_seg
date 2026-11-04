"""Mobile-crane Redis contract and host-side CX128S2 MSOP health monitor."""

import json
import socket
import threading
import time

from cx128s2_control import CX128S2Controller


LIDAR_MESSAGE_NORMAL = ""
LIDAR_MESSAGE_STANDBY = "standby"
LIDAR_MESSAGE_UPS_LOW_BATTERY = "ups_low_battery"
LIDAR_MESSAGE_PACKET_TIMEOUT = "lidar_packet_timeout"
LIDAR_MESSAGE_NETWORK_ERROR = "lidar_network_error"
LIDAR_MESSAGE_CONTROL_ERROR = "lidar_control_error"
LIDAR_MESSAGE_RECOVERY_FAILED = "lidar_recovery_failed"
LIDAR_MESSAGE_UNKNOWN_ERROR = "lidar_unknown_error"


def normalize_working_status(value):
    """Publish five FSM states; accept legacy API inputs during migration."""
    legacy = ("IDLE", "SUSPENDED", "LOWERING_CONFIRMATION", "LIFTING")
    if type(value) is int and 0 <= value < len(legacy):
        return legacy[value]
    if type(value) is str:
        value = value.upper()
        value = {"WORKING": "SUSPENDED", "DROP": "LOWERING_CONFIRMATION",
                 "COMPLETE_CONFIRM": "LOWERING_CONFIRMATION",
                 "LOWERING_CONFIRMED": "LOWERING_CONFIRMATION",
                 "LOWERING_CONFIRM": "LOWERING_CONFIRMATION"}.get(value, value)
        if value in ("IDLE", "LIFTING", "SUSPENDED", "LOWERING", "LOWERING_CONFIRMATION"):
            return value
    raise ValueError("working_status must be IDLE, LIFTING, SUSPENDED, LOWERING or LOWERING_CONFIRMATION")


class MobileCraneState:
    """Thread-safe source of truth for the six-key external payload contract."""

    def __init__(self):
        self._lock = threading.Lock()
        self._working_status = "IDLE"
        self._object_length = 0.0
        self._crane_moving = 0
        self._timestamp_ms = 0  # Sensor Unix epoch ms; 0 means no valid sensor stamp.
        self._lidar_status = 0
        self._lidar_message = LIDAR_MESSAGE_PACKET_TIMEOUT
        self._requested_lidar_state = "normal"
        self._normal_requested_at = None
        self._changed = threading.Event()

    def update_crane_state(self, working_status, object_length, crane_moving, sensor_timestamp_ms=None):
        working_status = normalize_working_status(working_status)
        if type(crane_moving) is not int or crane_moving not in (0, 1):
            raise ValueError("crane_moving must be 0 or 1")
        if isinstance(object_length, bool) or not isinstance(object_length, (int, float)) or object_length < 0:
            raise ValueError("object_length must be a non-negative number")
        if (sensor_timestamp_ms is not None and
                (type(sensor_timestamp_ms) is not int or sensor_timestamp_ms < 0)):
            raise ValueError("sensor_timestamp_ms must be a non-negative integer")
        with self._lock:
            self._working_status = working_status
            self._object_length = float(object_length)
            self._crane_moving = crane_moving
            self._timestamp_ms = 0 if sensor_timestamp_ms is None else sensor_timestamp_ms
        self._changed.set()

    def set_lidar_state(self, status, message):
        if status not in (0, 1):
            raise ValueError("lidar status must be 0 or 1")
        message = "" if message is None else message
        with self._lock:
            changed = (status, message) != (self._lidar_status, self._lidar_message)
            self._lidar_status, self._lidar_message = status, message
        if changed:
            self._changed.set()

    def request_lidar_state(self, mode, reason=None):
        with self._lock:
            self._requested_lidar_state = mode
            self._normal_requested_at = time.monotonic() if mode == "normal" else None
        if mode == "standby":
            self.set_lidar_state(0, reason or LIDAR_MESSAGE_STANDBY)

    def lidar_context(self):
        with self._lock:
            return self._requested_lidar_state, self._normal_requested_at

    def build_payload(self):
        """Return exactly the public mobile-crane Redis schema."""
        with self._lock:
            return {
                "working_status": self._working_status,
                "object_length": self._object_length,
                "crane_moving": self._crane_moving,
                "lidar_status": self._lidar_status,
                "lidar_message": self._lidar_message,
                "timestamp": self._timestamp_ms,
            }

    def consume_changed(self):
        changed = self._changed.is_set()
        self._changed.clear()
        return changed


class GeneralVehicleState:
    """Thread-safe source of truth for the general-mode vehicle Redis schema."""

    VALID_VEHICLE_STATUSES = {"entering", "stopping", "stopped", "exiting"}

    def __init__(self):
        self._lock = threading.Lock()
        self._vehicle_exist = 0
        # Keep a valid internal status; the public payload uses an empty string
        # and zero dimensions while no vehicle is present.
        self._vehicle_status = "stopped"
        self._vehicle_distance = 0.0
        self._vehicle_length = 0.0
        self._timestamp_ms = 0  # Last sensor time; Redis uses the current system time.
        self._lidar_status = 0
        self._lidar_message = LIDAR_MESSAGE_PACKET_TIMEOUT
        self._requested_lidar_state = "normal"
        self._normal_requested_at = None
        self._changed = threading.Event()

    def update_vehicle_state(self, vehicle_exist, vehicle_status, vehicle_distance, timestamp_ms,
                             vehicle_length):
        if type(vehicle_exist) is not int or vehicle_exist not in (0, 1):
            raise ValueError("vehicle_exist must be 0 or 1")
        if vehicle_status not in self.VALID_VEHICLE_STATUSES and not (vehicle_exist == 0 and vehicle_status == ""):
            raise ValueError("vehicle_status must be entering, stopping, stopped, or exiting")
        if (isinstance(vehicle_distance, bool) or
                not isinstance(vehicle_distance, (int, float)) or vehicle_distance < 0):
            raise ValueError("vehicle_distance must be a non-negative number")
        if (isinstance(vehicle_length, bool) or
                not isinstance(vehicle_length, (int, float)) or vehicle_length < 0):
            raise ValueError("vehicle_length must be a non-negative number")
        if type(timestamp_ms) is not int or timestamp_ms < 0:
            raise ValueError("timestamp_ms must be a non-negative integer")
        with self._lock:
            # A new sensor timestamp alone must not bypass the configured
            # periodic Redis publish interval; it is still retained for the
            # next transmission.
            changed = (vehicle_exist, vehicle_status, float(vehicle_distance), float(vehicle_length)) != (
                self._vehicle_exist, self._vehicle_status, self._vehicle_distance, self._vehicle_length)
            self._vehicle_exist = vehicle_exist
            self._vehicle_status = vehicle_status
            self._vehicle_distance = float(vehicle_distance)
            self._vehicle_length = float(vehicle_length)
            self._timestamp_ms = timestamp_ms
        if changed:
            self._changed.set()

    def set_lidar_state(self, status, message):
        if status not in (0, 1):
            raise ValueError("lidar status must be 0 or 1")
        message = "" if message is None else message
        with self._lock:
            changed = (status, message) != (self._lidar_status, self._lidar_message)
            self._lidar_status, self._lidar_message = status, message
        if changed:
            self._changed.set()

    def request_lidar_state(self, mode, reason=None):
        with self._lock:
            self._requested_lidar_state = mode
            self._normal_requested_at = time.monotonic() if mode == "normal" else None
        if mode == "standby":
            self.set_lidar_state(0, reason or LIDAR_MESSAGE_STANDBY)

    def lidar_context(self):
        with self._lock:
            return self._requested_lidar_state, self._normal_requested_at

    def build_payload(self):
        """Return the public general-mode Redis schema."""
        with self._lock:
            payload = {
                "vehicle_exist": self._vehicle_exist,
                "vehicle_status": self._vehicle_status if self._vehicle_exist else "",
                "vehicle_distance": self._vehicle_distance if self._vehicle_exist else 0.0,
                "vehicle_length": self._vehicle_length if self._vehicle_exist else 0.0,
                "lidar_status": self._lidar_status,
                "lidar_message": self._lidar_message,
                "timestamp": int(time.time() * 1000),
            }
            return payload

    def consume_changed(self):
        changed = self._changed.is_set()
        self._changed.clear()
        return changed


class CX128S2PacketMonitor:
    """Observes incoming MSOP UDP frames without binding the driver's port."""

    def __init__(self, lidar_ip, interface=None, msop_port=2368, timeout_sec=2.0):
        self.lidar_ip = lidar_ip
        self.interface = interface
        self.msop_port = msop_port
        self.timeout_sec = timeout_sec
        self._lock = threading.Lock()
        self._last_msop_time = None
        self._error = None
        self._thread = None
        self._stop = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._run, name="cx128s2-msop-monitor", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self):
        try:
            interface = self.interface
            if not interface:
                interface = CX128S2Controller(lidar_ip=self.lidar_ip).get_interface()
            with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003)) as raw_socket:
                raw_socket.bind((interface, 0))
                raw_socket.settimeout(1.0)
                while not self._stop.is_set():
                    try:
                        frame, _ = raw_socket.recvfrom(65535)
                    except socket.timeout:
                        continue
                    info = CX128S2Controller.parse_udp_frame(frame)
                    if info is None:
                        continue
                    if (info["src_ip"] == self.lidar_ip and info["dst_port"] == self.msop_port
                            and len(info["payload"]) > 0):
                        with self._lock:
                            self._last_msop_time = time.monotonic()
                            self._error = None
        except Exception as exc:
            with self._lock:
                self._error = str(exc)

    def health(self):
        with self._lock:
            last_msop_time, error = self._last_msop_time, self._error
        if error:
            return False, LIDAR_MESSAGE_NETWORK_ERROR
        if last_msop_time is None or time.monotonic() - last_msop_time > self.timeout_sec:
            return False, LIDAR_MESSAGE_PACKET_TIMEOUT
        return True, LIDAR_MESSAGE_NORMAL


class RedisPublisher:
    """Small dependency-free Redis Stream publisher; failures never raise to safety code."""

    def __init__(self, enabled=False, host="127.0.0.1", port=6379, key="guardian:mobile_crane_status", timeout=1.0, logger=None):
        self.enabled, self.host, self.port, self.key = enabled, host, port, key
        self.timeout, self.logger = timeout, logger
        self._last_error_log = 0.0

    @staticmethod
    def _resp(parts):
        encoded = [f"*{len(parts)}\r\n".encode()]
        for part in parts:
            value = part.encode("utf-8") if isinstance(part, str) else part
            encoded.extend((f"${len(value)}\r\n".encode(), value, b"\r\n"))
        return b"".join(encoded)

    def publish(self, payload):
        if not self.enabled:
            return True
        try:
            parts = ["XADD", self.key, "MAXLEN", "~", "86400", "*"]
            for field, value in payload.items():
                encoded_value = (value if isinstance(value, (str, bytes)) else
                                 json.dumps(value, separators=(",", ":"), allow_nan=False))
                parts.extend((str(field), encoded_value))
            with socket.create_connection((self.host, self.port), self.timeout) as connection:
                connection.settimeout(self.timeout)
                connection.sendall(self._resp(parts))
                with connection.makefile("rb") as reader:
                    response = reader.readline()
                    if not response.startswith(b"$"):
                        raise RuntimeError(f"unexpected Redis response: {response!r}")
                    length = int(response[1:-2])
                    stream_id = reader.read(length)
                    if length <= 0 or len(stream_id) != length or reader.read(2) != b"\r\n":
                        raise RuntimeError("incomplete Redis XADD response")
            return True
        except Exception as exc:
            if self.logger and time.monotonic() - self._last_error_log >= 30.0:
                self.logger("REDIS ERROR", str(exc))
                self._last_error_log = time.monotonic()
            return False
