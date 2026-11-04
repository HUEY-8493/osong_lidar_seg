#!/usr/bin/env python3

import os
import sys
import socket
import struct
import subprocess
import time
from pathlib import Path


LIDAR_IP = "192.168.1.200"
LIDAR_CONTROL_PORT = 2368

# CX128S2 UCWP
UCWP_HEADER = bytes.fromhex("AA 00 FF 11 22 22 AA AA")
STANDBY_OFFSET = 45

# Standby 후 motor_speed service가 동작하지 않을 수도 있으므로
# 정상 상태에서 캡처한 패킷을 저장해둔다.
CACHE_FILE = Path("/tmp/cx128s2_valid_ucwp_frame.bin")


def checksum(data: bytes) -> int:
    if len(data) % 2:
        data += b"\x00"

    total = 0

    for i in range(0, len(data), 2):
        total += (data[i] << 8) + data[i + 1]
        total = (total & 0xFFFF) + (total >> 16)

    return (~total) & 0xFFFF


def get_interface():
    result = subprocess.run(
        ["ip", "route", "get", LIDAR_IP],
        capture_output=True,
        text=True,
        check=True,
    )

    tokens = result.stdout.split()

    if "dev" not in tokens:
        raise RuntimeError(
            f"Could not determine interface:\n{result.stdout}"
        )

    iface = tokens[tokens.index("dev") + 1]

    return iface


def parse_udp_frame(frame):
    """
    Ethernet -> IPv4 -> UDP packet parser.

    Returns offsets and UDP payload.
    """

    if len(frame) < 14:
        return None

    ethertype = struct.unpack("!H", frame[12:14])[0]
    ip_offset = 14

    # VLAN
    if ethertype in (0x8100, 0x88A8):
        if len(frame) < 18:
            return None

        ethertype = struct.unpack("!H", frame[16:18])[0]
        ip_offset = 18

    if ethertype != 0x0800:
        return None

    if len(frame) < ip_offset + 20:
        return None

    version_ihl = frame[ip_offset]

    version = version_ihl >> 4
    ihl = (version_ihl & 0x0F) * 4

    if version != 4:
        return None

    if len(frame) < ip_offset + ihl:
        return None

    protocol = frame[ip_offset + 9]

    if protocol != 17:  # UDP
        return None

    src_ip = socket.inet_ntoa(
        frame[ip_offset + 12:ip_offset + 16]
    )

    dst_ip = socket.inet_ntoa(
        frame[ip_offset + 16:ip_offset + 20]
    )

    udp_offset = ip_offset + ihl

    if len(frame) < udp_offset + 8:
        return None

    src_port, dst_port, udp_length, udp_checksum = struct.unpack(
        "!HHHH",
        frame[udp_offset:udp_offset + 8]
    )

    payload_offset = udp_offset + 8
    payload_end = udp_offset + udp_length

    if payload_end > len(frame):
        return None

    payload = frame[payload_offset:payload_end]

    return {
        "ip_offset": ip_offset,
        "ip_header_length": ihl,
        "udp_offset": udp_offset,
        "udp_length": udp_length,
        "payload_offset": payload_offset,
        "payload": payload,
        "src_ip": src_ip,
        "dst_ip": dst_ip,
        "src_port": src_port,
        "dst_port": dst_port,
    }


def capture_valid_ucwp(iface):
    print()
    print("Capturing a valid UCWP packet from ROS driver...")
    print("This will call /ch/motor_speed with 10 Hz.")
    print()

    sock = socket.socket(
        socket.AF_PACKET,
        socket.SOCK_RAW,
        socket.htons(0x0003),
    )

    sock.bind((iface, 0))
    sock.settimeout(5.0)

    cmd = [
        "ros2",
        "service",
        "call",
        "/ch/motor_speed",
        "lslidar_msgs/srv/MotorSpeed",
        "{motor_speed: '10'}",
    ]

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    captured = None

    end_time = time.time() + 5.0

    while time.time() < end_time:

        try:
            frame, addr = sock.recvfrom(65535)
        except socket.timeout:
            break

        info = parse_udp_frame(frame)

        if info is None:
            continue

        if info["dst_ip"] != LIDAR_IP:
            continue

        if info["dst_port"] != LIDAR_CONTROL_PORT:
            continue

        payload = info["payload"]

        if len(payload) < 1206:
            continue

        if payload[:8] != UCWP_HEADER:
            continue

        captured = bytearray(frame)

        print("Captured valid UCWP packet:")
        print(f"  {info['src_ip']}:{info['src_port']}")
        print("       ->")
        print(f"  {info['dst_ip']}:{info['dst_port']}")
        print(f"  UCWP size : {len(payload)} bytes")
        print(f"  Motor RPM : {(payload[8] << 8) | payload[9]}")
        print(f"  Standby   : {payload[45]}")

        break

    stdout, stderr = proc.communicate(timeout=5)

    print()
    print(stdout.strip())

    if stderr.strip():
        print(stderr.strip())

    sock.close()

    if captured is None:
        raise RuntimeError(
            "Could not capture UCWP packet.\n"
            "Make sure the ROS driver is running and "
            "/ch/motor_speed works."
        )

    CACHE_FILE.write_bytes(captured)

    print()
    print(f"Cached valid packet: {CACHE_FILE}")

    return captured


def recalc_checksums(frame):
    info = parse_udp_frame(frame)

    if info is None:
        raise RuntimeError("Invalid Ethernet/IP/UDP frame")

    ip_offset = info["ip_offset"]
    ihl = info["ip_header_length"]
    udp_offset = info["udp_offset"]
    udp_length = info["udp_length"]

    #
    # IPv4 checksum
    #

    frame[ip_offset + 10] = 0
    frame[ip_offset + 11] = 0

    ip_header = bytes(
        frame[ip_offset:ip_offset + ihl]
    )

    ip_cksum = checksum(ip_header)

    frame[ip_offset + 10:ip_offset + 12] = struct.pack(
        "!H",
        ip_cksum,
    )

    #
    # UDP checksum
    #

    frame[udp_offset + 6] = 0
    frame[udp_offset + 7] = 0

    src_ip = frame[
        ip_offset + 12:
        ip_offset + 16
    ]

    dst_ip = frame[
        ip_offset + 16:
        ip_offset + 20
    ]

    pseudo_header = (
        bytes(src_ip)
        + bytes(dst_ip)
        + b"\x00"
        + b"\x11"
        + struct.pack("!H", udp_length)
    )

    udp_data = bytes(
        frame[
            udp_offset:
            udp_offset + udp_length
        ]
    )

    udp_cksum = checksum(
        pseudo_header + udp_data
    )

    if udp_cksum == 0:
        udp_cksum = 0xFFFF

    frame[
        udp_offset + 6:
        udp_offset + 8
    ] = struct.pack(
        "!H",
        udp_cksum,
    )


def patch_standby(frame, value):
    info = parse_udp_frame(frame)

    if info is None:
        raise RuntimeError("Invalid packet")

    payload_offset = info["payload_offset"]
    payload = info["payload"]

    if payload[:8] != UCWP_HEADER:
        raise RuntimeError("Not a CX128S2 UCWP packet")

    before = frame[
        payload_offset + STANDBY_OFFSET
    ]

    frame[
        payload_offset + STANDBY_OFFSET
    ] = value

    recalc_checksums(frame)

    print()
    print("UCWP modification:")
    print(f"  standby: {before} -> {value}")
    print(
        "  motor rpm:",
        (frame[payload_offset + 8] << 8)
        | frame[payload_offset + 9],
    )


def send_raw_frame(iface, frame):
    sock = socket.socket(
        socket.AF_PACKET,
        socket.SOCK_RAW,
    )

    sock.bind((iface, 0))

    sent = sock.send(frame)

    sock.close()

    print()
    print(f"Sent {sent} byte Ethernet frame.")


def main():

    if os.geteuid() != 0:
        print("ERROR: run as root")
        print()
        print(
            "sudo -E python3 "
            "cx128s2_standby_test.py standby"
        )
        sys.exit(1)

    if len(sys.argv) != 2:
        print(
            f"Usage: {sys.argv[0]} "
            "standby|normal"
        )
        sys.exit(1)

    mode = sys.argv[1].lower()

    if mode not in ("standby", "normal"):
        print("Mode must be standby or normal")
        sys.exit(1)

    value = 1 if mode == "standby" else 0

    iface = get_interface()

    print(f"Interface : {iface}")
    print(f"LiDAR     : {LIDAR_IP}")
    print(f"Mode      : {mode}")

    #
    # STANDBY:
    # Capture a fresh known-good packet.
    #
    # NORMAL:
    # Prefer cached packet because in standby
    # the driver may no longer receive enough
    # information for setUcwpData().
    #

    if mode == "standby":

        frame = capture_valid_ucwp(iface)

    else:

        if CACHE_FILE.exists():

            print()
            print(
                f"Using cached UCWP packet: "
                f"{CACHE_FILE}"
            )

            frame = bytearray(
                CACHE_FILE.read_bytes()
            )

        else:

            print()
            print(
                "No cached packet found."
            )

            frame = capture_valid_ucwp(iface)

    patch_standby(
        frame,
        value,
    )

    print()
    print(
        f"Sending CX128S2 "
        f"{mode.upper()} command..."
    )

    send_raw_frame(
        iface,
        frame,
    )

    print()
    print("Done.")


if __name__ == "__main__":
    main()