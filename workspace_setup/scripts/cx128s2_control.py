#!/usr/bin/env python3
"""Host-side, cache-backed CX128S2 UCWP standby controller.

The controller deliberately sends a previously captured *complete* driver
frame.  It never manufactures a UCWP configuration: only byte 45 (standby)
and the enclosing IPv4/UDP checksums are changed.
"""

import os
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path


LIDAR_IP = "192.168.1.200"
LIDAR_CONTROL_PORT = 2368
UCWP_HEADER = bytes.fromhex("AA 00 FF 11 22 22 AA AA")
DIFOP_HEADER = bytes.fromhex("A5 FF 00 5A")
UCWP_SIZE = 1206
STANDBY_OFFSET = 45
DEFAULT_CACHE_FILE = Path("/var/lib/guardianai/cx128s2_valid_ucwp_frame.bin")


def checksum(data: bytes) -> int:
    if len(data) % 2:
        data += b"\x00"
    total = 0
    for offset in range(0, len(data), 2):
        total += (data[offset] << 8) + data[offset + 1]
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


class CX128S2Controller:
    """Serializes host raw-frame control commands for one CX128S2."""

    def __init__(self, lidar_ip=LIDAR_IP, interface=None, cache_file=DEFAULT_CACHE_FILE):
        self.lidar_ip = lidar_ip
        self.interface_override = interface
        self.cache_file = Path(cache_file)
        self._lock = threading.Lock()
        self._last_command = None
        self._last_result = None

    def get_interface(self):
        if self.interface_override:
            return self.interface_override
        result = subprocess.run(
            ["ip", "route", "get", self.lidar_ip],
            capture_output=True, text=True, timeout=2.0, check=False,
        )
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or "ip route get failed")
        fields = result.stdout.split()
        try:
            return fields[fields.index("dev") + 1]
        except (ValueError, IndexError) as exc:
            raise RuntimeError("Could not determine LiDAR interface") from exc

    @staticmethod
    def parse_udp_frame(frame):
        if len(frame) < 14:
            return None
        ethertype = struct.unpack("!H", frame[12:14])[0]
        ip_offset = 14
        if ethertype in (0x8100, 0x88A8):
            if len(frame) < 18:
                return None
            ethertype = struct.unpack("!H", frame[16:18])[0]
            ip_offset = 18
        if ethertype != 0x0800 or len(frame) < ip_offset + 20:
            return None
        version_ihl = frame[ip_offset]
        ihl = (version_ihl & 0x0F) * 4
        if version_ihl >> 4 != 4 or ihl < 20 or len(frame) < ip_offset + ihl:
            return None
        if frame[ip_offset + 9] != 17:
            return None
        udp_offset = ip_offset + ihl
        if len(frame) < udp_offset + 8:
            return None
        src_port, dst_port, udp_length, _ = struct.unpack("!HHHH", frame[udp_offset:udp_offset + 8])
        payload_offset = udp_offset + 8
        payload_end = udp_offset + udp_length
        if udp_length < 8 or payload_end > len(frame):
            return None
        return {
            "ip_offset": ip_offset, "ip_header_length": ihl,
            "udp_offset": udp_offset, "udp_length": udp_length,
            "payload_offset": payload_offset, "payload": frame[payload_offset:payload_end],
            "src_ip": socket.inet_ntoa(frame[ip_offset + 12:ip_offset + 16]),
            "dst_ip": socket.inet_ntoa(frame[ip_offset + 16:ip_offset + 20]),
            "src_port": src_port, "dst_port": dst_port,
        }

    def _validate_frame(self, frame, require_normal=False):
        info = self.parse_udp_frame(frame)
        if info is None:
            raise RuntimeError("Invalid Ethernet/IPv4/UDP frame")
        if info["dst_ip"] != self.lidar_ip or info["dst_port"] != LIDAR_CONTROL_PORT:
            raise RuntimeError("Cached frame does not target this LiDAR control endpoint")
        payload = info["payload"]
        if len(payload) != UCWP_SIZE or payload[:8] != UCWP_HEADER:
            raise RuntimeError("Frame does not contain a valid 1206-byte CX128S2 UCWP packet")
        if require_normal and payload[STANDBY_OFFSET] != 0:
            raise RuntimeError("Refusing to cache a non-Normal UCWP packet")
        return info

    def _store_frame(self, frame):
        self.cache_file.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
        temporary = self.cache_file.with_suffix(self.cache_file.suffix + ".tmp")
        temporary.write_bytes(frame)
        os.chmod(temporary, 0o600)
        temporary.replace(self.cache_file)

    def _capture_reference_from_difop(self, timeout):
        """Build a safe normal UCWP Ethernet frame from live LiDAR DIFOP.

        This is the same configuration-preserving operation used by
        LslidarServices::setUcwpData(): copy all 1206 DIFOP bytes and replace
        only the UCWP header.  The received frame is reversed at Ethernet/IP
        level so it is sent back from this host to the LiDAR.
        """
        interface = self.get_interface()
        with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003)) as raw_socket:
            raw_socket.bind((interface, 0))
            raw_socket.settimeout(min(timeout, 1.0))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    frame, _ = raw_socket.recvfrom(65535)
                except socket.timeout:
                    continue
                info = self.parse_udp_frame(frame)
                if info is None:
                    continue
                payload = info["payload"]
                if (info["src_ip"] != self.lidar_ip or len(payload) != UCWP_SIZE
                        or payload[:4] != DIFOP_HEADER):
                    continue
                converted = bytearray(frame)
                # Incoming: LiDAR -> host. Outgoing: host -> LiDAR.
                converted[0:6], converted[6:12] = frame[6:12], frame[0:6]
                ip_offset = info["ip_offset"]
                converted[ip_offset + 12:ip_offset + 16] = frame[ip_offset + 16:ip_offset + 20]
                converted[ip_offset + 16:ip_offset + 20] = frame[ip_offset + 12:ip_offset + 16]
                udp_offset = info["udp_offset"]
                converted[udp_offset:udp_offset + 4] = struct.pack(
                    "!HH", info["dst_port"], LIDAR_CONTROL_PORT
                )
                payload_offset = info["payload_offset"]
                converted[payload_offset:payload_offset + 8] = UCWP_HEADER
                converted[payload_offset + STANDBY_OFFSET] = 0
                converted_info = self._validate_frame(converted, require_normal=True)
                self._recalculate_checksums(converted, converted_info)
                self._store_frame(converted)
                return interface
        raise RuntimeError("No valid CX128S2 DIFOP packet observed before timeout")

    def capture_reference_packet(self, timeout=5.0):
        """Explicit DIFOP bootstrap command; normally commands bootstrap automatically."""
        try:
            interface = self._capture_reference_from_difop(timeout)
            return self._result(True, "capture", interface=interface, cached_packet=True, source="difop")
        except Exception as exc:
            return self._result(False, "capture", str(exc))

    @staticmethod
    def _recalculate_checksums(frame, info):
        ip_offset, ihl = info["ip_offset"], info["ip_header_length"]
        udp_offset, udp_length = info["udp_offset"], info["udp_length"]
        frame[ip_offset + 10:ip_offset + 12] = b"\x00\x00"
        frame[ip_offset + 10:ip_offset + 12] = struct.pack("!H", checksum(bytes(frame[ip_offset:ip_offset + ihl])))
        frame[udp_offset + 6:udp_offset + 8] = b"\x00\x00"
        pseudo_header = (bytes(frame[ip_offset + 12:ip_offset + 20]) + b"\x00\x11" + struct.pack("!H", udp_length))
        udp_checksum = checksum(pseudo_header + bytes(frame[udp_offset:udp_offset + udp_length]))
        frame[udp_offset + 6:udp_offset + 8] = struct.pack("!H", udp_checksum or 0xFFFF)

    def _command(self, requested_mode):
        standby_value = 1 if requested_mode == "standby" else 0
        with self._lock:
            try:
                if not self.cache_file.is_file():
                    # No ROS/Docker control call is made here. A live DIFOP is
                    # sufficient to preserve the LiDAR's current configuration.
                    self._capture_reference_from_difop(timeout=5.0)
                frame = bytearray(self.cache_file.read_bytes())
                info = self._validate_frame(frame, require_normal=True)
                interface = self.get_interface()
                frame[info["payload_offset"] + STANDBY_OFFSET] = standby_value
                self._recalculate_checksums(frame, info)
                with socket.socket(socket.AF_PACKET, socket.SOCK_RAW) as raw_socket:
                    raw_socket.bind((interface, 0))
                    sent = raw_socket.send(frame)
                if sent != len(frame):
                    raise RuntimeError(f"Short raw-frame send ({sent}/{len(frame)} bytes)")
                self._last_command = requested_mode
                return self._result(True, requested_mode, interface=interface, bytes_sent=sent)
            except Exception as exc:
                return self._result(False, requested_mode, str(exc))

    def standby(self):
        return self._command("standby")

    def normal(self):
        return self._command("normal")

    def _result(self, success, requested_mode, error=None, **extra):
        result = {
            "success": success, "lidar_ip": self.lidar_ip,
            "requested_mode": requested_mode, "cached_packet": self.cache_file.is_file(),
            "last_command": self._last_command, "verified": False,
        }
        if error:
            result["error"] = error
        result.update(extra)
        self._last_result = result
        return result

    def status(self):
        try:
            interface, interface_error = self.get_interface(), None
        except Exception as exc:
            interface, interface_error = None, str(exc)
        result = self._result(True, "status", interface=interface)
        if interface_error:
            result.update(success=False, error=interface_error)
        # No DIFOP acknowledgement is read here; requested and actual state
        # remain intentionally distinct.
        result["reported_mode"] = None
        result["reachable"] = None  # no network probe is treated as verification
        return result
