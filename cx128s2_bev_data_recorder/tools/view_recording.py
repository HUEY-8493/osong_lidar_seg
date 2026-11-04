#!/usr/bin/env python3
"""Inspect a cx128s2_bev_data_recorder directory without ROS or Open3D."""
import argparse
import csv
import struct
import sys
from pathlib import Path


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def read_xyz_pcd(path):
    with path.open("rb") as stream:
        header = b""
        while b"DATA " not in header:
            line = stream.readline()
            if not line:
                raise ValueError(f"Invalid PCD header: {path}")
            header += line
        text = header.decode("ascii")
        points = int(next(x.split()[1] for x in text.splitlines() if x.startswith("POINTS ")))
        binary = "DATA binary" in text
        if binary:
            raw = stream.read(points * 12)
            return [struct.unpack_from("<fff", raw, offset) for offset in range(0, len(raw), 12)]
        return [tuple(map(float, line.split()[:3])) for line in stream if line.strip()]


def frame_objects(objects, frame_id):
    return [row for row in objects if int(row["frame_id"]) == int(frame_id)]


def plot_frame(ax, frame, objects, root):
    points = read_xyz_pcd(root / frame["pcd_file"])
    ax.clear()
    if points:
        xs, ys, zs = zip(*points)
        ax.scatter(xs, ys, zs, s=0.5, c="0.35")
    for obj in frame_objects(objects, frame["frame_id"]):
        x, y, z = float(obj["x"]), float(obj["y"]), float(obj["z"])
        ax.scatter([x], [y], [z], s=45, c="tab:red")
        ax.text(x, y, z, f'ID {obj["track_id"]} {float(obj["speed"]):.2f}m/s\n{obj["recording_status"]}', fontsize=8)
    ax.set_title(f'frame {frame["frame_id"]}  stamp {frame["timestamp_sec"]}')
    ax.set_xlabel("X (forward)"); ax.set_ylabel("Y (left)"); ax.set_zlabel("Z (up)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("recording_directory", type=Path)
    parser.add_argument("--frame", type=int, default=0, help="zero-based row index")
    parser.add_argument("--play", action="store_true", help="animate all recorded frames")
    parser.add_argument("--track", type=int, help="plot X/Y/speed timeline for this track ID")
    parser.add_argument("--interval-ms", type=int, default=100)
    args = parser.parse_args()
    root = args.recording_directory
    frames, objects = read_csv(root / "frames.csv"), read_csv(root / "objects.csv")
    if not frames:
        sys.exit("No recorded frames.")
    import matplotlib.pyplot as plt
    if args.track is not None:
        rows = [r for r in objects if int(r["track_id"]) == args.track]
        if not rows:
            sys.exit(f"Track {args.track} is not in objects.csv")
        t0 = int(rows[0]["timestamp_ns"])
        t = [(int(r["timestamp_ns"]) - t0) / 1e9 for r in rows]
        fig, axes = plt.subplots(3, 1, sharex=True)
        axes[0].plot(t, [float(r["x"]) for r in rows], label="x")
        axes[0].plot(t, [float(r["y"]) for r in rows], label="y")
        axes[1].plot(t, [float(r["speed"]) for r in rows], label="speed")
        axes[2].step(t, [r["recording_status"] for r in rows], where="post")
        axes[0].legend(); axes[1].legend(); axes[2].set_xlabel("seconds")
        axes[0].set_ylabel("position (m)"); axes[1].set_ylabel("speed (m/s)"); axes[2].set_ylabel("motion")
        plt.show(); return
    fig = plt.figure(); ax = fig.add_subplot(111, projection="3d")
    if args.play:
        from matplotlib.animation import FuncAnimation
        animation = FuncAnimation(fig, lambda i: plot_frame(ax, frames[i], objects, root),
                                  frames=len(frames), interval=args.interval_ms, repeat=True)
        _ = animation  # retain it until pyplot closes
    else:
        if not 0 <= args.frame < len(frames): sys.exit(f"--frame must be in [0, {len(frames)-1}]")
        plot_frame(ax, frames[args.frame], objects, root)
    plt.show()


if __name__ == "__main__":
    main()
