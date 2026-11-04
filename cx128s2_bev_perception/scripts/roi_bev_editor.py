#!/usr/bin/env python3
"""2D BEV ROI policy editor for cx128s2_bev_perception."""

import argparse
import threading
import os
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons, TextBox
import numpy as np
import yaml

def resolve_path(path):
    if not path:
        return path

    path = os.path.expanduser(path)

    if path.startswith("package://"):
        from ament_index_python.packages import get_package_share_directory

        rest = path[len("package://"):]
        parts = rest.split("/", 1)
        if len(parts) != 2 or not parts[0] or not parts[1]:
            raise RuntimeError(f"Invalid package:// path: {path}")

        package_name, relative_path = parts
        pkg_share = get_package_share_directory(package_name)
        return os.path.join(pkg_share, relative_path)

    # /파일명 shortcut
    if path.startswith("/") and len(path) > 1:
        rest = path[1:]

        # /tmp/..., /home/... 같은 실제 절대경로는 유지
        if "/" not in rest:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("cx128s2_bev_perception")
            return os.path.join(pkg_share, "config", rest)

        return path

    # 상대 파일명 shortcut
    if "/" not in path:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory("cx128s2_bev_perception")
        return os.path.join(pkg_share, "config", path)

    return path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from sensor_msgs_py import point_cloud2
    from sensor_msgs.msg import PointCloud2
except Exception:  # ROS is optional for loading/editing saved ROI YAML.
    rclpy = None
    Node = object
    QoSDurabilityPolicy = None
    QoSHistoryPolicy = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    point_cloud2 = None
    PointCloud2 = None


COLORS = {
    "watch": "limegreen",
    "ignore": "gold",
    "hard_ignore": "red",
}


def str_to_bool(value):
    if isinstance(value, bool):
        return value
    lowered = str(value).strip().lower()
    if lowered in ("1", "true", "yes", "y", "on"):
        return True
    if lowered in ("0", "false", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {value}")


def world_to_canvas(x, y):
    # world/ROS coordinates -> displayed canvas coordinates.
    # Default BEV projection: horizontal canvas axis is world Y, vertical axis is world X.
    return y, x


def canvas_to_world(canvas_x, canvas_y):
    # displayed canvas coordinates -> world/ROS coordinates.
    # Default inverse of world_to_canvas(): world X is canvas Y, world Y is canvas X.
    return canvas_y, canvas_x


@dataclass
class BevProjection:
    forward_axis: str = "x"
    lateral_axis: str = "y"
    swap_xy: bool = True
    invert_y_axis: bool = False

    def _axis_value(self, axis, x, y):
        return x if axis == "x" else y

    def _axis_set(self, axis, value, x, y):
        if axis == "x":
            return value, y
        return x, value

    def world_to_canvas(self, x, y):
        forward = self._axis_value(self.forward_axis, x, y)
        lateral = self._axis_value(self.lateral_axis, x, y)
        if self.invert_y_axis:
            lateral = -lateral
        if self.swap_xy:
            return lateral, forward
        return forward, lateral

    def canvas_to_world(self, canvas_x, canvas_y):
        if self.swap_xy:
            lateral = canvas_x
            forward = canvas_y
        else:
            forward = canvas_x
            lateral = canvas_y
        if self.invert_y_axis:
            lateral = -lateral
        x, y = 0.0, 0.0
        x, y = self._axis_set(self.forward_axis, forward, x, y)
        x, y = self._axis_set(self.lateral_axis, lateral, x, y)
        return x, y

    def canvas_limits(self, x_min, x_max, y_min, y_max):
        corners = [
            self.world_to_canvas(x_min, y_min),
            self.world_to_canvas(x_min, y_max),
            self.world_to_canvas(x_max, y_min),
            self.world_to_canvas(x_max, y_max),
        ]
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        return min(xs), max(xs), min(ys), max(ys)

    def description(self):
        if self.swap_xy:
            canvas_x = f"world {self.lateral_axis.upper()}"
            canvas_y = f"world {self.forward_axis.upper()}"
            if self.invert_y_axis:
                canvas_x = "-" + canvas_x
        else:
            canvas_x = f"world {self.forward_axis.upper()}"
            canvas_y = f"world {self.lateral_axis.upper()}"
            if self.invert_y_axis:
                canvas_y = "-" + canvas_y
        return f"Canvas X = {canvas_x}; Canvas Y = {canvas_y}; Saved ROI = world X/Y"


@dataclass
class RoiZone:
    name: str
    type: str
    shape: str
    enabled: bool = True
    x_min: float = 0.0
    x_max: float = 0.0
    y_min: float = 0.0
    y_max: float = 0.0
    polygon_x: list = field(default_factory=list)
    polygon_y: list = field(default_factory=list)

    def vertices(self):
        if self.shape == "box":
            return (
                [self.x_min, self.x_max, self.x_max, self.x_min, self.x_min],
                [self.y_min, self.y_min, self.y_max, self.y_max, self.y_min],
            )
        xs = list(self.polygon_x)
        ys = list(self.polygon_y)
        if xs:
            xs.append(xs[0])
            ys.append(ys[0])
        return xs, ys

    def to_yaml(self):
        data = {
            "name": self.name,
            "type": self.type,
            "shape": self.shape,
            "enabled": bool(self.enabled),
        }
        if self.shape == "box":
            data.update({
                "x_min": float(self.x_min),
                "x_max": float(self.x_max),
                "y_min": float(self.y_min),
                "y_max": float(self.y_max),
            })
        else:
            data["polygon_x"] = [float(v) for v in self.polygon_x]
            data["polygon_y"] = [float(v) for v in self.polygon_y]
        return data


class CloudSubscriber(Node):
    def __init__(self, topic, z_min, z_max, max_points, qos_profile, cloud_mode):
        super().__init__("roi_bev_editor")
        self.z_min = z_min
        self.z_max = z_max
        self.max_points = max_points
        self.cloud_mode = cloud_mode
        self.lock = threading.Lock()
        self.points = ([], [], [], [])
        self.point_seq = 0
        self.accept_next_snapshot = True
        self.last_cloud_stamp = ""
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos_profile)
        self.get_logger().info(f"Subscribed to {topic} with QoS: {qos_profile}, mode={cloud_mode}")

    def on_cloud(self, msg):
        with self.lock:
            if self.cloud_mode == "snapshot" and not self.accept_next_snapshot:
                return
            self.accept_next_snapshot = False
        xs, ys, zs, intensities = [], [], [], []
        step = 1
        try:
            total = msg.width * msg.height
            if self.max_points > 0 and total > self.max_points:
                step = max(1, total // self.max_points)
            field_names = [f.name for f in msg.fields]
            read_intensity = "intensity" in field_names
            read_fields = ("x", "y", "z", "intensity") if read_intensity else ("x", "y", "z")
            for i, p in enumerate(point_cloud2.read_points(
                    msg, field_names=read_fields, skip_nans=True)):
                if i % step:
                    continue
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                if z < self.z_min or z > self.z_max:
                    continue
                xs.append(x)
                ys.append(y)
                zs.append(z)
                intensities.append(float(p[3]) if read_intensity else 0.0)
        except Exception as exc:
            self.get_logger().warn(f"PointCloud2 read failed: {exc}")
            return
        with self.lock:
            self.points = (xs, ys, zs, intensities)
            self.point_seq += 1
            stamp = msg.header.stamp
            self.last_cloud_stamp = f"{stamp.sec}.{stamp.nanosec:09d}"
        self.get_logger().info(
            f"Captured point cloud snapshot: kept={len(xs)} total={msg.width * msg.height} "
            f"seq={self.point_seq}")

    def latest_points(self):
        with self.lock:
            return self.point_seq, self.last_cloud_stamp, tuple(list(v) for v in self.points)

    def request_snapshot(self):
        with self.lock:
            self.accept_next_snapshot = True


class RoiEditor:
    def __init__(self, args, ros_node=None):
        self.args = args
        self.ros_node = ros_node
        self.projection = BevProjection(
            forward_axis=args.bev_forward_axis,
            lateral_axis=args.bev_lateral_axis,
            swap_xy=args.swap_xy,
            invert_y_axis=args.invert_y_axis,
        )
        self.zones = []
        self.current_points = []
        self.current_type = "watch"
        self.current_shape = "polygon"
        self.current_name = "roi_1"
        self.last_point_seq = -1
        self.drawing_enabled = False
        self.buttons = []

        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.fig.subplots_adjust(left=0.07, right=0.80, bottom=0.18)
        self.ax.set_aspect("equal", adjustable="box")
        c_x_min, c_x_max, c_y_min, c_y_max = self.projection.canvas_limits(
            args.x_min, args.x_max, args.y_min, args.y_max)
        self.ax.set_xlim(c_x_min, c_x_max)
        self.ax.set_ylim(c_y_min, c_y_max)
        if args.swap_xy and args.bev_forward_axis == "x" and args.bev_lateral_axis == "y":
            horizontal = "-Y lateral" if args.invert_y_axis else "Y lateral"
            self.ax.set_xlabel(f"{horizontal} [m]")
            self.ax.set_ylabel("X forward [m]")
        else:
            self.ax.set_xlabel("Canvas X [m]")
            self.ax.set_ylabel("Canvas Y [m]")
        self.ax.grid(True, which="both")
        self.ax.set_xticks(np.arange(np.floor(c_x_min / 10.0) * 10.0, c_x_max + 10.0, 10.0))
        self.ax.set_yticks(np.arange(np.floor(c_y_min / 10.0) * 10.0, c_y_max + 10.0, 10.0))
        self.ax.set_title("BEV Top-Down View (X forward, Y lateral)")

        self.scatter = self.ax.scatter([], [], s=1, c="0.25", alpha=0.35, cmap="viridis")
        self.coord_text = self.ax.text(0.01, 0.99, "", transform=self.ax.transAxes,
                                       va="top", ha="left")
        self.cloud_text = self.ax.text(0.99, 0.99, "Cloud: waiting for snapshot",
                                       transform=self.ax.transAxes, va="top", ha="right",
                                       bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "0.7"})
        self.draw_text = self.ax.text(
            0.99, 0.90, "Draw: OFF (press Draw or d)",
            transform=self.ax.transAxes,
            va="top",
            ha="right",
            bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "0.7"},
        )
        self.projection_text = self.ax.text(
            0.01, 0.01,
            "Projection: BEV XY top-down\n"
            f"{self.projection.description()}\n"
            "+X forward; horizontal sign follows projection option\n\n"
            "Recommended Topics:\n"
            "• /debug/roi_filtered_points (ROI filtered = actual perception input)\n"
            "• /debug/valid_points (Valid = before ROI)\n"
            "• /debug/foreground_points (Foreground = after background subtraction)",
            transform=self.ax.transAxes,
            va="bottom",
            ha="left",
            bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "0.7"},
        )
        self.zone_artists = []
        self.overlay_artists = []
        self.current_artist = None
        self.draw_axis_overlay()
        self.setup_widgets()
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        if args.input:
            self.load_yaml(args.input)
        self.timer = self.fig.canvas.new_timer(interval=args.refresh_ms)
        self.timer.add_callback(self.refresh)
        self.timer.start()

    def setup_widgets(self):
        type_ax = self.fig.add_axes([0.82, 0.70, 0.15, 0.16])
        self.type_radio = RadioButtons(type_ax, ("watch", "ignore", "hard_ignore"))
        self.type_radio.on_clicked(self.set_type)

        shape_ax = self.fig.add_axes([0.82, 0.55, 0.15, 0.10])
        self.shape_radio = RadioButtons(shape_ax, ("polygon", "box"))
        self.shape_radio.on_clicked(self.set_shape)

        name_ax = self.fig.add_axes([0.82, 0.47, 0.15, 0.05])
        self.name_box = TextBox(name_ax, "name", initial=self.current_name)
        self.name_box.on_submit(self.set_name)

        buttons = [
            ("Draw: Off", self.toggle_draw_mode),
            ("Snapshot", self.request_snapshot),
            ("Finish", self.finish_roi),
            ("Undo", self.undo_point),
            ("Delete Last", self.delete_last),
            ("Save", self.save_yaml),
            ("Load", lambda _event: self.load_yaml(self.args.output)),
        ]
        y = 0.38
        for label, callback in buttons:
            bax = self.fig.add_axes([0.82, y, 0.15, 0.045])
            button = Button(bax, label)
            button.on_clicked(callback)
            self.buttons.append(button)
            if label.startswith("Draw"):
                self.draw_button = button
            y -= 0.06

    def set_type(self, label):
        self.current_type = label

    def set_shape(self, label):
        self.current_shape = label
        self.current_points = []
        self.redraw()

    def set_name(self, text):
        self.current_name = text.strip() or f"roi_{len(self.zones) + 1}"

    def on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        toolbar = getattr(self.fig.canvas, "toolbar", None)
        if toolbar is not None and getattr(toolbar, "mode", ""):
            return
        if not self.drawing_enabled:
            return
        canvas_point = (float(event.xdata), float(event.ydata))
        self.current_points.append(canvas_point)
        wx, wy = self.canvas_to_world(*canvas_point)
        print(f"Added ROI point: canvas=({canvas_point[0]:.3f}, {canvas_point[1]:.3f}) "
              f"world: x={wx:.3f}m, y={wy:.3f}m")
        if self.current_shape == "box" and len(self.current_points) >= 2:
            self.finish_roi(None)
        else:
            self.redraw()

    def on_motion(self, event):
        if event.inaxes == self.ax and event.xdata is not None and event.ydata is not None:
            wx, wy = self.canvas_to_world(float(event.xdata), float(event.ydata))
            self.coord_text.set_text(
                f"canvas=({event.xdata:.2f}, {event.ydata:.2f}), "
                f"world: x={wx:.2f}m, y={wy:.2f}m")
            self.fig.canvas.draw_idle()

    def on_key(self, event):
        if event.key in ("enter", "f"):
            self.finish_roi(None)
        elif event.key in ("backspace", "u"):
            self.undo_point(None)
        elif event.key == "delete":
            self.delete_last(None)
        elif event.key == "s":
            self.save_yaml(None)
        elif event.key == "l":
            self.load_yaml(self.args.output)
        elif event.key == "r":
            self.request_snapshot(None)
        elif event.key == "d":
            self.toggle_draw_mode(None)

    def set_draw_mode(self, enabled):
        self.drawing_enabled = bool(enabled)
        label = "Draw: On" if self.drawing_enabled else "Draw: Off"
        text = "Draw: ON (click to add ROI points)" if self.drawing_enabled else "Draw: OFF (pan/zoom safe)"
        if hasattr(self, "draw_button"):
            self.draw_button.label.set_text(label)
        self.draw_text.set_text(text)
        self.fig.canvas.draw_idle()

    def toggle_draw_mode(self, _event):
        self.set_draw_mode(not self.drawing_enabled)
        print("ROI drawing mode:", "ON" if self.drawing_enabled else "OFF")

    def request_snapshot(self, _event):
        if self.ros_node is None:
            print("No ROS subscriber is active; cannot capture a new snapshot.")
            return
        self.ros_node.request_snapshot()
        self.cloud_text.set_text("Cloud: waiting for next snapshot")
        self.fig.canvas.draw_idle()
        print("Requested one new PointCloud2 snapshot")

    def finish_roi(self, _event):
        if self.current_shape == "polygon" and len(self.current_points) < 3:
            print("Need at least 3 points for polygon ROI")
            return
        if self.current_shape == "box" and len(self.current_points) < 2:
            print("Need 2 corner points for box ROI")
            return
        name = self.current_name or f"roi_{len(self.zones) + 1}"
        world_points = [self.canvas_to_world(px, py) for px, py in self.current_points]
        if self.current_shape == "box":
            xs = [p[0] for p in world_points[:2]]
            ys = [p[1] for p in world_points[:2]]
            zone = RoiZone(name, self.current_type, "box", True,
                           min(xs), max(xs), min(ys), max(ys))
        else:
            zone = RoiZone(name, self.current_type, "polygon", True,
                           polygon_x=[p[0] for p in world_points],
                           polygon_y=[p[1] for p in world_points])
        self.zones.append(zone)
        print(f"Finished ROI '{zone.name}' ({zone.type}, {zone.shape}) in world X/Y coordinates")
        self.current_points = []
        self.current_name = f"roi_{len(self.zones) + 1}"
        self.name_box.set_val(self.current_name)
        self.set_draw_mode(False)
        self.redraw()

    def undo_point(self, _event):
        if self.current_points:
            self.current_points.pop()
            self.redraw()

    def delete_last(self, _event):
        if self.zones:
            removed = self.zones.pop()
            print(f"Deleted ROI: {removed.name}")
            self.redraw()

    def save_yaml(self, _event):
        data = {
            "roi_policy": {
                "version": 1,
                "frame_id": self.args.frame,
                "zones": [z.to_yaml() for z in self.zones],
            }
        }
        with open(self.args.output, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        print(f"Saved {len(self.zones)} ROI zones to {self.args.output}")

    def load_yaml(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            print(f"ROI file not found: {path}")
            return
        policy = data.get("roi_policy", data)
        zones = []
        for item in policy.get("zones", []):
            zones.append(RoiZone(
                name=item.get("name", "roi"),
                type=item.get("type", "watch"),
                shape=item.get("shape", "polygon"),
                enabled=bool(item.get("enabled", True)),
                x_min=float(item.get("x_min", 0.0)),
                x_max=float(item.get("x_max", 0.0)),
                y_min=float(item.get("y_min", 0.0)),
                y_max=float(item.get("y_max", 0.0)),
                polygon_x=list(item.get("polygon_x", [])),
                polygon_y=list(item.get("polygon_y", [])),
            ))
        self.zones = zones
        print(f"Loaded {len(self.zones)} ROI zones from {path}")
        self.redraw()

    def refresh(self):
        if self.ros_node is not None:
            seq, stamp, points = self.ros_node.latest_points()
            if seq == self.last_point_seq:
                return True
            self.last_point_seq = seq
            xs, ys, zs, intensities = points
            if xs:
                canvas = [self.world_to_canvas(x, y) for x, y in zip(xs, ys)]
                offsets = np.asarray(canvas)
                if self.args.color_by == "z":
                    self.scatter.set_array(np.asarray(zs))
                    self.scatter.set_cmap("viridis")
                elif self.args.color_by == "intensity":
                    self.scatter.set_array(np.asarray(intensities))
                    self.scatter.set_cmap("magma")
                else:
                    self.scatter.set_array(None)
                    self.scatter.set_color("0.25")
                self.scatter.set_alpha(0.45 if self.args.color_by in ("z", "intensity") else 0.35)
            else:
                offsets = np.empty((0, 2))
                self.scatter.set_array(None)
            self.scatter.set_offsets(offsets)
            mode = "snapshot" if self.args.cloud_mode == "snapshot" else "live"
            self.cloud_text.set_text(
                f"Cloud: {mode} seq={seq} points={len(xs)}\nstamp={stamp or 'n/a'}")
            self.redraw()
            return True
        self.redraw()
        return True

    def world_to_canvas(self, x, y):
        return self.projection.world_to_canvas(x, y)

    def canvas_to_world(self, canvas_x, canvas_y):
        return self.projection.canvas_to_world(canvas_x, canvas_y)

    def zone_canvas_vertices(self, zone):
        xs, ys = zone.vertices()
        if not xs:
            return [], []
        points = [self.world_to_canvas(x, y) for x, y in zip(xs, ys)]
        return [p[0] for p in points], [p[1] for p in points]

    def draw_axis_overlay(self):
        for artist in self.overlay_artists:
            artist.remove()
        self.overlay_artists = []
        ox, oy = self.world_to_canvas(0.0, 0.0)
        x1, y1 = self.world_to_canvas(10.0, 0.0)
        x2, y2 = self.world_to_canvas(0.0, 10.0)
        origin = self.ax.scatter([ox], [oy], s=40, marker="+", c="black", zorder=5)
        x_arrow = self.ax.annotate(
            "+X forward", xy=(x1, y1), xytext=(ox, oy),
            arrowprops={"arrowstyle": "->", "color": "tab:blue", "lw": 2},
            color="tab:blue")
        y_arrow = self.ax.annotate(
            "+Y lateral", xy=(x2, y2), xytext=(ox, oy),
            arrowprops={"arrowstyle": "->", "color": "tab:orange", "lw": 2},
            color="tab:orange")
        origin_label = self.ax.text(ox, oy, "  origin (0,0)", color="black", va="bottom")
        self.overlay_artists.extend([origin, x_arrow, y_arrow, origin_label])

    def redraw(self):
        for artist in self.zone_artists:
            artist.remove()
        self.zone_artists = []
        for zone in self.zones:
            xs, ys = self.zone_canvas_vertices(zone)
            if not xs:
                continue
            color = COLORS.get(zone.type, "white")
            line, = self.ax.plot(xs, ys, color=color, linewidth=2.0)
            text = self.ax.text(sum(xs) / len(xs), sum(ys) / len(ys),
                                f"{zone.name}\n{zone.type}", color=color)
            self.zone_artists.extend([line, text])
        if self.current_points:
            xs = [p[0] for p in self.current_points]
            ys = [p[1] for p in self.current_points]
            if self.current_shape == "box" and len(self.current_points) == 2:
                xs = [min(xs), max(xs), max(xs), min(xs), min(xs)]
                ys = [min(ys), min(ys), max(ys), max(ys), min(ys)]
            line, = self.ax.plot(xs, ys, color=COLORS.get(self.current_type, "white"),
                                 linestyle="--", marker="o")
            self.zone_artists.append(line)
        self.fig.canvas.draw_idle()


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "XY top-down BEV ROI editor. --x-min/--x-max are world X forward/backward "
            "limits; --y-min/--y-max are world Y lateral limits. ROI YAML is always saved "
            "in world X/Y coordinates used by the C++ perception node."
        )
    )
    parser.add_argument("--topic", default="/debug/roi_filtered_points")
    parser.add_argument("--output", default="/tmp/cx128s2_roi_policy.yaml")
    parser.add_argument("--input", default="")
    parser.add_argument("--frame", default="world")
    parser.add_argument("--x-min", type=float, default=-20.0,
                        help="World X minimum [m], usually backward/behind sensor.")
    parser.add_argument("--x-max", type=float, default=220.0,
                        help="World X maximum [m], usually forward.")
    parser.add_argument("--y-min", type=float, default=-120.0,
                        help="World Y minimum [m], lateral.")
    parser.add_argument("--y-max", type=float, default=120.0,
                        help="World Y maximum [m], lateral.")
    parser.add_argument("--z-min", type=float, default=-5.0)
    parser.add_argument("--z-max", type=float, default=8.0)
    parser.add_argument("--max-points", type=int, default=20000)
    parser.add_argument("--refresh-ms", type=int, default=300)
    parser.add_argument(
        "--cloud-mode",
        choices=("snapshot", "live"),
        default="snapshot",
        help="snapshot captures one PointCloud2 frame and ignores later clouds until Snapshot/r is pressed.",
    )
    parser.add_argument("--bev-forward-axis", choices=("x", "y"), default="x",
                        help="World axis treated as forward in the BEV display.")
    parser.add_argument("--bev-lateral-axis", choices=("x", "y"), default="y",
                        help="World axis treated as lateral in the BEV display.")
    parser.add_argument("--invert-y-axis", type=str_to_bool, default=False,
                        help="Invert displayed lateral sign. Default false: canvas X = world Y.")
    parser.add_argument("--swap-xy", type=str_to_bool, default=True,
                        help="Default true: canvas X=lateral, canvas Y=forward.")
    parser.add_argument(
        "--color-by",
        choices=("fixed", "z", "intensity", "source_topic"),
        default="fixed",
        help="Use z/intensity only for coloring; never as a display axis.",
    )
    parser.add_argument(
        "--qos",
        choices=("sensor_data", "best_effort", "reliable", "default", "system_default"),
        default="sensor_data",
        help="PointCloud2 subscription QoS preset. sensor_data/best_effort works with most LiDAR drivers.",
    )
    parser.add_argument("--qos-depth", type=int, default=10)
    parser.add_argument(
        "--qos-reliability",
        choices=("auto", "best_effort", "reliable", "system_default"),
        default="auto",
        help="Override reliability for the selected QoS preset.",
    )
    parser.add_argument(
        "--qos-durability",
        choices=("auto", "volatile", "transient_local", "system_default"),
        default="auto",
        help="Override durability for the selected QoS preset.",
    )
    parser.add_argument("--no-ros", action="store_true")
    args = parser.parse_args()
    if args.bev_forward_axis == args.bev_lateral_axis:
        parser.error("--bev-forward-axis and --bev-lateral-axis must be different")
    return args


def make_qos_profile(args):
    depth = max(1, int(args.qos_depth))
    if args.qos in ("sensor_data", "best_effort"):
        reliability = QoSReliabilityPolicy.BEST_EFFORT
        durability = QoSDurabilityPolicy.VOLATILE
    elif args.qos == "reliable":
        reliability = QoSReliabilityPolicy.RELIABLE
        durability = QoSDurabilityPolicy.VOLATILE
    elif args.qos == "system_default":
        reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
        durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
    else:
        reliability = QoSReliabilityPolicy.RELIABLE
        durability = QoSDurabilityPolicy.VOLATILE

    reliability_overrides = {
        "best_effort": QoSReliabilityPolicy.BEST_EFFORT,
        "reliable": QoSReliabilityPolicy.RELIABLE,
        "system_default": QoSReliabilityPolicy.SYSTEM_DEFAULT,
    }
    durability_overrides = {
        "volatile": QoSDurabilityPolicy.VOLATILE,
        "transient_local": QoSDurabilityPolicy.TRANSIENT_LOCAL,
        "system_default": QoSDurabilityPolicy.SYSTEM_DEFAULT,
    }
    if args.qos_reliability != "auto":
        reliability = reliability_overrides[args.qos_reliability]
    if args.qos_durability != "auto":
        durability = durability_overrides[args.qos_durability]

    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=durability,
    )


def main():
    args = parse_args()
    try:
        args.output = resolve_path(args.output)
        if args.input:
            args.input = resolve_path(args.input)
    except Exception as exc:
        print(f"Path resolution failed: {exc}")
        return 1
    ros_node = None
    if not args.no_ros and rclpy is not None:
        rclpy.init()
        ros_node = CloudSubscriber(
            args.topic,
            args.z_min,
            args.z_max,
            args.max_points,
            make_qos_profile(args),
            args.cloud_mode,
        )
        thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
        thread.start()
    elif not args.no_ros:
        print("ROS2 Python modules are unavailable; starting editor without live PointCloud2.")
    editor = RoiEditor(args, ros_node)
    try:
        plt.show()
    finally:
        if ros_node is not None:
            ros_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
