#!/usr/bin/env python3
"""
One-shot LiDAR ground-plane calibration from binary/ascii PCD files.

Estimates only:
  - z / height
  - roll
  - pitch

Manual inputs:
  - x
  - y
  - yaw

Frame convention:
  - x: forward
  - y: left
  - z: up
"""

import argparse
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    here = Path.cwd()
    parser = argparse.ArgumentParser(
        description='One-shot LiDAR roll/pitch/height calibration from PCD ground plane.'
    )
    parser.add_argument('--pcd-dir', default=str(here / 'pcd'), help='Directory containing .pcd files.')
    parser.add_argument('--manual-x', type=float, default=0.0, help='Manual LiDAR x position [m].')
    parser.add_argument('--manual-y', type=float, default=3.0, help='Manual LiDAR y position [m].')
    parser.add_argument('--manual-yaw-deg', type=float, default=0.0, help='Manual LiDAR yaw [deg].')
    parser.add_argument('--ground-z', type=float, default=0.0, help='World ground z [m].')
    parser.add_argument('--sample-count', type=int, default=30, help='Number of PCD frames to use.')
    parser.add_argument('--max-points-per-frame', type=int, default=50000)
    parser.add_argument('--z-min-filter', type=float, default=-50.0, help='Min filter range (default -50.0m).')
    parser.add_argument('--z-max-filter', type=float, default=50.0, help='Max filter range (default 50.0m).')
    parser.add_argument('--ransac-distance-threshold', type=float, default=0.03)
    parser.add_argument('--ransac-max-iterations', type=int, default=1000)
    parser.add_argument('--min-ground-points', type=int, default=1000)
    parser.add_argument('--random-seed', type=int, default=7)
    parser.add_argument(
        '--output-yaml-path',
        default=str(here / 'lidar_ground_calibration.yaml'),
        help='Output YAML path.',
    )
    parser.add_argument(
        '--debug-plot-path',
        default=str(here / 'lidar_ground_calibration_debug.png'),
        help='Debug plot PNG path.',
    )
    parser.add_argument(
        '--view-plot-path',
        default=str(here / 'lidar_scan_calibration_applied_view.png'),
        help='Applied scan view PNG path.',
    )
    parser.add_argument(
        '--first-person-plot-path',
        default=str(here / 'lidar_first_person_view.png'),
        help='LiDAR first-person view PNG path.',
    )
    parser.add_argument('--view-max-points', type=int, default=35000, help='Max points in applied scan view.')
    parser.add_argument('--no-debug-plot', action='store_true', help='Disable debug plot generation.')
    parser.add_argument('--no-view-plot', action='store_true', help='Disable applied scan view generation.')
    parser.add_argument('--no-first-person-plot', action='store_true', help='Disable LiDAR first-person view.')
    return parser.parse_args()


def pcd_field_dtype(size: int, type_code: str, count: int):
    endian = '<'
    if type_code == 'F':
        base = {4: 'f4', 8: 'f8'}[size]
    elif type_code == 'I':
        base = {1: 'i1', 2: 'i2', 4: 'i4', 8: 'i8'}[size]
    elif type_code == 'U':
        base = {1: 'u1', 2: 'u2', 4: 'u4', 8: 'u8'}[size]
    else:
        raise ValueError(f'Unsupported PCD TYPE {type_code!r}')

    dtype = np.dtype(endian + base)
    if count == 1:
        return dtype
    return (dtype, (count,))


def read_pcd_xyz(path: Path) -> np.ndarray:
    """Read x/y/z columns from ascii or binary PCD without Open3D/PCL."""
    raw = path.read_bytes()
    header_end = raw.find(b'DATA ')
    if header_end < 0:
        raise ValueError(f'{path}: missing DATA line')
    data_line_end = raw.find(b'\n', header_end)
    if data_line_end < 0:
        raise ValueError(f'{path}: malformed DATA line')

    header_text = raw[:data_line_end].decode('utf-8', errors='replace')
    metadata: Dict[str, List[str]] = {}
    for line in header_text.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        metadata[parts[0].upper()] = parts[1:]

    fields = metadata['FIELDS']
    sizes = [int(v) for v in metadata['SIZE']]
    types = metadata['TYPE']
    counts = [int(v) for v in metadata.get('COUNT', ['1'] * len(fields))]
    points = int(metadata.get('POINTS', metadata.get('WIDTH', ['0']))[0])
    data_kind = metadata['DATA'][0].lower()
    payload = raw[data_line_end + 1:]

    for axis in ('x', 'y', 'z'):
        if axis not in fields:
            raise ValueError(f'{path}: missing {axis!r} field')

    if data_kind == 'ascii':
        table = np.loadtxt(payload.decode('utf-8', errors='replace').splitlines(), dtype=np.float64)
        index = [fields.index(axis) for axis in ('x', 'y', 'z')]
        return table[:, index]

    if data_kind != 'binary':
        raise ValueError(f'{path}: only ascii and binary PCD are supported, got {data_kind!r}')

    dtype = np.dtype(
        [
            (field, pcd_field_dtype(size, type_code, count))
            for field, size, type_code, count in zip(fields, sizes, types, counts)
        ]
    )
    cloud = np.frombuffer(payload, dtype=dtype, count=points)
    return np.column_stack([cloud['x'], cloud['y'], cloud['z']]).astype(np.float64)


def filter_points(
    points: np.ndarray,
    z_min: float,
    z_max: float,
    max_points: int,
    rng: np.random.Generator,
) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    mask = np.isfinite(points).all(axis=1)
    mask &= (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    filtered = points[mask]
    if max_points > 0 and filtered.shape[0] > max_points:
        filtered = filtered[rng.choice(filtered.shape[0], size=max_points, replace=False)]
    return filtered


def normalize_plane(normal: np.ndarray, d: float) -> Tuple[np.ndarray, float]:
    normal = np.asarray(normal, dtype=np.float64)
    norm = float(np.linalg.norm(normal))
    if norm < 1.0e-12:
        raise ValueError('Plane normal is degenerate.')
    return normal / norm, float(d) / norm


def orient_plane_towards_sensor(normal: np.ndarray, d: float) -> Tuple[np.ndarray, float]:
    """Orient the normal vector so it points from the ground towards the sensor origin (d >= 0)."""
    normal, d = normalize_plane(normal, d)
    if d < 0.0:
        return -normal, -d
    return normal, d


def plane_from_three_points(sample: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    v1 = sample[1] - sample[0]
    v2 = sample[2] - sample[0]
    normal = np.cross(v1, v2)
    if np.linalg.norm(normal) < 1.0e-9:
        return None
    d = -float(np.dot(normal, sample[0]))
    return orient_plane_towards_sensor(normal, d)


def refine_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, float]:
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    d = -float(np.dot(normal, centroid))
    return orient_plane_towards_sensor(normal, d)


def fit_plane_ransac(
    points: np.ndarray,
    distance_threshold: float,
    max_iterations: int,
    rng: np.random.Generator,
) -> Optional[dict]:
    if points.shape[0] < 3:
        return None

    best_mask = None
    best_count = -1
    best_median = float('inf')

    for _ in range(max_iterations):
        sample = points[rng.choice(points.shape[0], size=3, replace=False)]
        plane = plane_from_three_points(sample)
        if plane is None:
            continue
        normal, d = plane
        residuals = np.abs(points @ normal + d)
        mask = residuals <= distance_threshold
        count = int(mask.sum())
        if count == 0:
            continue
        median = float(np.median(residuals[mask]))
        if count > best_count or (count == best_count and median < best_median):
            best_mask = mask
            best_count = count
            best_median = median

    if best_mask is None or best_count < 3:
        return None

    normal, d = refine_plane_svd(points[best_mask])
    residuals = np.abs(points @ normal + d)
    inlier_mask = residuals <= distance_threshold
    inlier_residuals = residuals[inlier_mask]

    return {
        'normal': normal,
        'd': d,
        'inlier_mask': inlier_mask,
        'inlier_count': int(inlier_mask.sum()),
        'residual_mean': float(np.mean(inlier_residuals)) if inlier_residuals.size else float('nan'),
        'residual_median': float(np.median(inlier_residuals)) if inlier_residuals.size else float('nan'),
        'inlier_residuals': inlier_residuals,
    }


def robust_average_plane(planes: Iterable[Tuple[np.ndarray, float]]) -> Tuple[np.ndarray, float]:
    """Average planes across frames with consistent normal direction."""
    normals = []
    ds = []
    ref_normal = None
    for normal, d in planes:
        normal, d = orient_plane_towards_sensor(normal, d)
        if ref_normal is None:
            ref_normal = normal
        elif np.dot(normal, ref_normal) < 0.0:
            normal = -normal
            d = -d
        normals.append(normal)
        ds.append(d)
    if not normals:
        raise RuntimeError('No valid plane estimates were produced.')
    avg_normal = np.median(np.vstack(normals), axis=0)
    avg_d = float(np.median(ds))
    return orient_plane_towards_sensor(avg_normal, avg_d)


def roll_pitch_from_ground_normal(normal: np.ndarray) -> Tuple[float, float]:
    """Return ROS roll/pitch for R = Rz(yaw)Ry(pitch)Rx(roll).

    The third row of R is:
      [-sin(pitch), cos(pitch)sin(roll), cos(pitch)cos(roll)]

    Setting this row equal to the LiDAR-frame ground normal n=[nx,ny,nz]
    makes transformed ground points satisfy world_z = n^T p_lidar + z.
    Yaw does not appear in this row, so yaw is not observable from a plane.
    """
    normal, _ = normalize_plane(normal, 0.0)
    nx, ny, nz = normal
    pitch = math.asin(float(np.clip(-nx, -1.0, 1.0)))
    roll = math.atan2(float(ny), float(nz))
    return roll, pitch


def select_plane_orientation_for_positive_height(
    normal: np.ndarray,
    d: float,
    ground_z: float,
) -> Tuple[np.ndarray, float, float]:
    """Choose the plane sign that maps ground to ground_z with positive LiDAR z.

    For p_world = R p_lidar + t and RPY R = Rz Ry Rx:
      world_z = r3 dot p_lidar + t_z

    If the LiDAR-frame ground plane is n dot p + d = 0 and r3 = n,
    then all ground points satisfy:
      world_z = -d + t_z

    Therefore t_z = ground_z + d. Since a plane has two equivalent signs,
    (n, d) and (-n, -d), we choose the sign that makes t_z positive.
    """
    normal, d = normalize_plane(normal, d)
    candidates = [(normal, d), (-normal, -d)]
    candidates.sort(key=lambda item: (ground_z + item[1] <= ground_z, abs(ground_z + item[1])))
    selected_normal, selected_d = candidates[0]
    return selected_normal, selected_d, ground_z + selected_d


def rotation_matrix_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def save_debug_plot(z_error: np.ndarray, residuals: np.ndarray, threshold: float, path: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    axes[0].hist(z_error, bins=80, color='#2563eb', alpha=0.85)
    axes[0].axvline(0.0, color='black', linewidth=1.0)
    axes[0].set_title('Ground inlier world z error')
    axes[0].set_xlabel('world_z - ground_z [m]')
    axes[0].set_ylabel('count')

    axes[1].hist(residuals, bins=80, color='#d97706', alpha=0.85)
    axes[1].axvline(threshold, color='black', linewidth=1.0)
    axes[1].set_title('Plane residual')
    axes[1].set_xlabel('|a*x + b*y + c*z + d| [m]')
    axes[1].set_ylabel('count')

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_applied_scan_view(
    points_lidar: np.ndarray,
    normal: np.ndarray,
    d: float,
    threshold: float,
    roll: float,
    pitch: float,
    yaw: float,
    translation: np.ndarray,
    ground_z: float,
    validation: dict,
    yaml_path: Path,
    pcd_path: Path,
    path: Path,
    max_points: int,
    rng: np.random.Generator,
) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    points_lidar = np.asarray(points_lidar, dtype=np.float64).reshape((-1, 3))
    if max_points > 0 and points_lidar.shape[0] > max_points:
        points_lidar = points_lidar[rng.choice(points_lidar.shape[0], size=max_points, replace=False)]

    rotation = rotation_matrix_rpy(roll, pitch, yaw)
    world = points_lidar @ rotation.T + translation
    residual = np.abs(points_lidar @ normal + d)
    inlier = residual <= threshold
    z_color = np.clip(world[:, 2], ground_z - 3.0, ground_z + 16.0)

    path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(16, 10), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, height_ratios=[1, 1], width_ratios=[1, 1])
    ax_xy = fig.add_subplot(grid[0, 0])
    ax_xz = fig.add_subplot(grid[0, 1])
    ax_yz = fig.add_subplot(grid[1, 0])
    ax_info = fig.add_subplot(grid[1, 1])

    scatter = ax_xy.scatter(
        world[:, 0], world[:, 1], c=z_color, s=0.7, cmap='viridis', alpha=0.65, rasterized=True
    )
    ax_xy.scatter([translation[0]], [translation[1]], c='red', s=80, marker='*', label='LiDAR position')
    ax_xy.arrow(
        translation[0], translation[1], rotation[0, 0] * 3.0, rotation[1, 0] * 3.0,
        color='red', width=0.08, length_includes_head=True
    )
    ax_xy.arrow(
        translation[0], translation[1], rotation[0, 1] * 3.0, rotation[1, 1] * 3.0,
        color='orange', width=0.08, length_includes_head=True
    )
    ax_xy.text(
        translation[0] + rotation[0, 0] * 3.2,
        translation[1] + rotation[1, 0] * 3.2,
        'LiDAR +x forward',
        color='red',
    )
    ax_xy.text(
        translation[0] + rotation[0, 1] * 3.2,
        translation[1] + rotation[1, 1] * 3.2,
        'LiDAR +y left',
        color='darkorange',
    )
    ax_xy.set_title('Top view: world x-y, colored by world z')
    ax_xy.set_xlabel('world x [m]')
    ax_xy.set_ylabel('world y [m]')
    ax_xy.axis('equal')
    ax_xy.grid(True, alpha=0.25)
    ax_xy.legend(loc='best')

    ax_xz.scatter(world[:, 0], world[:, 2], c=z_color, s=0.7, cmap='viridis', alpha=0.55, rasterized=True)
    if inlier.any():
        ax_xz.scatter(world[inlier, 0], world[inlier, 2], c='magenta', s=0.9, alpha=0.35, label='RANSAC inliers')
    ax_xz.axhline(ground_z, color='black', linewidth=2.0, label=f'ground_z = {ground_z:g}')
    ax_xz.scatter([translation[0]], [translation[2]], c='red', s=90, marker='*', label='LiDAR')
    ax_xz.set_title('Side view: world x-z')
    ax_xz.set_xlabel('world x [m]')
    ax_xz.set_ylabel('world z [m]')
    ax_xz.grid(True, alpha=0.25)
    ax_xz.legend(loc='best')

    ax_yz.scatter(world[:, 1], world[:, 2], c=z_color, s=0.7, cmap='viridis', alpha=0.55, rasterized=True)
    if inlier.any():
        ax_yz.scatter(world[inlier, 1], world[inlier, 2], c='magenta', s=0.9, alpha=0.35, label='RANSAC inliers')
    ax_yz.axhline(ground_z, color='black', linewidth=2.0, label=f'ground_z = {ground_z:g}')
    ax_yz.scatter([translation[1]], [translation[2]], c='red', s=90, marker='*', label='LiDAR')
    ax_yz.set_title('Side view: world y-z')
    ax_yz.set_xlabel('world y [m]')
    ax_yz.set_ylabel('world z [m]')
    ax_yz.grid(True, alpha=0.25)
    ax_yz.legend(loc='best')

    colorbar = fig.colorbar(scatter, ax=[ax_xy, ax_xz, ax_yz], fraction=0.025, pad=0.02)
    colorbar.set_label(f'world z [m], clipped [{ground_z - 3:g}, {ground_z + 16:g}]')

    warning = ''
    if abs(validation['median_z_error']) > 2.0 * threshold:
        warning = (
            '\nWarning:\n'
            '  RANSAC inliers are not near ground_z after transform.\n'
            '  Adjust ROI/filtering before trusting this calibration.\n'
        )

    info = f"""
Applied transform from:
  {yaml_path.name}

LiDAR position:
  x = {translation[0]:.3f} m
  y = {translation[1]:.3f} m
  z = {translation[2]:.3f} m

Orientation:
  roll  = {math.degrees(roll):.3f} deg
  pitch = {math.degrees(pitch):.3f} deg
  yaw   = {math.degrees(yaw):.3f} deg (manual)

Fitted plane in LiDAR frame:
  n = [{normal[0]:+.4f}, {normal[1]:+.4f}, {normal[2]:+.4f}]
  d = {d:+.4f}

Validation:
  mean z error   = {validation['mean_z_error']:.3f} m
  median z error = {validation['median_z_error']:.3f} m
  max |z error|  = {validation['max_abs_z_error']:.3f} m
  std z error    = {validation['std_z_error']:.3f} m

Reading:
  black line = world ground_z
  red star = LiDAR origin
  magenta = RANSAC plane inliers
{warning}
""".strip()
    ax_info.axis('off')
    ax_info.text(0.02, 0.98, info, va='top', ha='left', family='monospace', fontsize=10)

    fig.suptitle(f'LiDAR scan with applied calibration: {pcd_path.name}', fontsize=15)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def save_lidar_first_person_view(
    points_lidar: np.ndarray,
    normal: np.ndarray,
    d: float,
    threshold: float,
    pcd_path: Path,
    path: Path,
    max_points: int,
    rng: np.random.Generator,
) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    points_lidar = np.asarray(points_lidar, dtype=np.float64).reshape((-1, 3))
    if max_points > 0 and points_lidar.shape[0] > max_points:
        points_lidar = points_lidar[rng.choice(points_lidar.shape[0], size=max_points, replace=False)]

    x = points_lidar[:, 0]
    y = points_lidar[:, 1]
    z = points_lidar[:, 2]
    distance = np.linalg.norm(points_lidar, axis=1)
    residual = np.abs(points_lidar @ normal + d)
    inlier = residual <= threshold

    azimuth = np.degrees(np.arctan2(y, x))
    xy_range = np.hypot(x, y)
    elevation = np.degrees(np.arctan2(z, xy_range))

    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)
    ax_yz, ax_angle, ax_top, ax_info = axes.ravel()

    distance_color = np.clip(distance, 0.0, np.nanpercentile(distance, 99.0))

    sc = ax_yz.scatter(y, z, c=distance_color, s=0.8, cmap='turbo', alpha=0.62, rasterized=True)
    if inlier.any():
        ax_yz.scatter(y[inlier], z[inlier], c='magenta', s=1.1, alpha=0.38, label='RANSAC ground inliers')
    ax_yz.axvline(0.0, color='black', linewidth=1.0, alpha=0.7)
    ax_yz.axhline(0.0, color='black', linewidth=1.0, alpha=0.7)
    ax_yz.set_title('LiDAR first-person image plane: y-z')
    ax_yz.set_xlabel('LiDAR y [m]  (+ left)')
    ax_yz.set_ylabel('LiDAR z [m]  (+ up)')
    ax_yz.grid(True, alpha=0.25)
    ax_yz.legend(loc='best')

    ax_angle.scatter(azimuth, elevation, c=distance_color, s=0.8, cmap='turbo', alpha=0.62, rasterized=True)
    if inlier.any():
        ax_angle.scatter(azimuth[inlier], elevation[inlier], c='magenta', s=1.1, alpha=0.38)
    ax_angle.axvline(0.0, color='black', linewidth=1.0, alpha=0.7)
    ax_angle.axhline(0.0, color='black', linewidth=1.0, alpha=0.7)
    ax_angle.set_title('LiDAR angular view')
    ax_angle.set_xlabel('azimuth atan2(y, x) [deg]')
    ax_angle.set_ylabel('elevation atan2(z, hypot(x,y)) [deg]')
    ax_angle.grid(True, alpha=0.25)

    ax_top.scatter(x, y, c=distance_color, s=0.8, cmap='turbo', alpha=0.62, rasterized=True)
    if inlier.any():
        ax_top.scatter(x[inlier], y[inlier], c='magenta', s=1.1, alpha=0.38)
    ax_top.scatter([0.0], [0.0], c='red', s=80, marker='*', label='LiDAR origin')
    ax_top.arrow(0.0, 0.0, 4.0, 0.0, color='red', width=0.08, length_includes_head=True)
    ax_top.arrow(0.0, 0.0, 0.0, 4.0, color='orange', width=0.08, length_includes_head=True)
    ax_top.text(4.3, 0.0, '+x forward', color='red')
    ax_top.text(0.2, 4.3, '+y left', color='darkorange')
    ax_top.set_title('LiDAR native top view: x-y')
    ax_top.set_xlabel('LiDAR x [m]  (+ forward)')
    ax_top.set_ylabel('LiDAR y [m]  (+ left)')
    ax_top.axis('equal')
    ax_top.grid(True, alpha=0.25)
    ax_top.legend(loc='best')

    colorbar = fig.colorbar(sc, ax=[ax_yz, ax_angle, ax_top], fraction=0.025, pad=0.02)
    colorbar.set_label('range from LiDAR [m], clipped at 99 percentile')

    info = f"""
LiDAR first-person view
Source:
  {pcd_path.name}

Frame convention used by this script:
  x = forward
  y = left
  z = up

Panels:
  y-z: camera-like first-person plane
  angular: azimuth/elevation scan image
  x-y: native LiDAR top view

Colors:
  range from LiDAR origin
  magenta = RANSAC ground inliers

Current fitted plane:
  n = [{normal[0]:+.4f}, {normal[1]:+.4f}, {normal[2]:+.4f}]
  d = {d:+.4f}

Note:
  If left/right looks reversed here, the PCD frame
  is not using y-left as assumed, or the sensor driver
  exported a different axis convention.
""".strip()
    ax_info.axis('off')
    ax_info.text(0.02, 0.98, info, va='top', ha='left', family='monospace', fontsize=10)

    fig.suptitle('LiDAR first-person view', fontsize=15)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    rng = np.random.default_rng(args.random_seed)
    pcd_dir = Path(args.pcd_dir).expanduser().resolve()
    output_yaml_path = Path(args.output_yaml_path).expanduser().resolve()
    debug_plot_path = Path(args.debug_plot_path).expanduser().resolve()
    view_plot_path = Path(args.view_plot_path).expanduser().resolve()
    first_person_plot_path = Path(args.first_person_plot_path).expanduser().resolve()

    pcd_files = sorted(pcd_dir.glob('*.pcd'))[:args.sample_count]
    if not pcd_files:
        raise FileNotFoundError(f'No .pcd files found in {pcd_dir}')

    planes = []
    ground_inliers = []
    residual_sets = []
    inlier_counts = []
    used_files = []

    for index, pcd_path in enumerate(pcd_files, start=1):
        points = read_pcd_xyz(pcd_path)
        filtered = filter_points(
            points,
            args.z_min_filter,
            args.z_max_filter,
            args.max_points_per_frame,
            rng,
        )
        if filtered.shape[0] < args.min_ground_points:
            print(
                f'[WARN] skip {pcd_path.name}: valid points {filtered.shape[0]} '
                f'< min_ground_points {args.min_ground_points}'
            )
            continue

        fit = fit_plane_ransac(
            filtered,
            args.ransac_distance_threshold,
            args.ransac_max_iterations,
            rng,
        )
        if fit is None or fit['inlier_count'] < args.min_ground_points:
            count = 0 if fit is None else fit['inlier_count']
            print(
                f'[WARN] skip {pcd_path.name}: RANSAC inliers {count} '
                f'< min_ground_points {args.min_ground_points}'
            )
            continue

        planes.append((fit['normal'], fit['d']))
        ground_inliers.append(filtered[fit['inlier_mask']])
        residual_sets.append(fit['inlier_residuals'])
        inlier_counts.append(fit['inlier_count'])
        used_files.append(str(pcd_path))

        n = fit['normal']
        print(
            f'[{len(planes):02d}/{args.sample_count:02d}] {pcd_path.name}: '
            f'n=[{n[0]:+.5f}, {n[1]:+.5f}, {n[2]:+.5f}], '
            f'd={fit["d"]:+.5f}, inliers={fit["inlier_count"]}, '
            f'residual_mean={fit["residual_mean"]:.5f}, '
            f'residual_median={fit["residual_median"]:.5f}'
        )

    if not planes:
        raise RuntimeError('Calibration failed: no valid ground planes were found.')

    normal, d = robust_average_plane(planes)
    normal, d, final_z = select_plane_orientation_for_positive_height(normal, d, args.ground_z)
    height = final_z - args.ground_z
    final_x = args.manual_x
    final_y = args.manual_y
    yaw = math.radians(args.manual_yaw_deg)
    roll, pitch = roll_pitch_from_ground_normal(normal)

    all_inliers = np.vstack(ground_inliers)
    residuals = np.concatenate(residual_sets)
    rotation = rotation_matrix_rpy(roll, pitch, yaw)
    world_z = all_inliers @ rotation[2, :].T + final_z
    z_error = world_z - args.ground_z

    validation = {
        'mean_z_error': float(np.mean(z_error)),
        'median_z_error': float(np.median(z_error)),
        'max_abs_z_error': float(np.max(np.abs(z_error))),
        'std_z_error': float(np.std(z_error)),
    }
    inlier_count_median = float(np.median(np.asarray(inlier_counts)))

    if height < 0.2 or height > 10.0:
        print(f'[WARN] estimated height={height:.3f} m is outside a typical LiDAR mounting range.')
    if abs(validation['median_z_error']) > 2.0 * args.ransac_distance_threshold:
        print(
            f'[WARN] median_z_error={validation["median_z_error"]:.4f} exceeds '
            f'2x threshold={2.0 * args.ransac_distance_threshold:.4f}.'
        )

    result = {
        'lidar_extrinsic': {
            'parent_frame': 'map',
            'child_frame': 'lidar',
            'frame_convention': {
                'x': 'forward',
                'y': 'left',
                'z': 'up',
            },
            'x': float(final_x),
            'y': float(final_y),
            'z': float(final_z),
            'roll': float(roll),
            'pitch': float(pitch),
            'yaw': float(yaw),
            'roll_deg': float(math.degrees(roll)),
            'pitch_deg': float(math.degrees(pitch)),
            'yaw_deg': float(args.manual_yaw_deg),
            'calibration': {
                'method': 'ground_plane_ransac_manual_x_y_yaw',
                'ground_z': float(args.ground_z),
                'plane_normal_lidar': [float(v) for v in normal],
                'plane_d': float(d),
                'used_frames': int(len(planes)),
                'used_files': used_files,
                'inlier_count_median': float(inlier_count_median),
                'ransac_distance_threshold': float(args.ransac_distance_threshold),
            },
            'validation': validation,
            'manual_inputs': {
                'manual_x': float(args.manual_x),
                'manual_y': float(args.manual_y),
                'manual_yaw_deg': float(args.manual_yaw_deg),
                'manual_y_recommended_range_m': [2.5, 3.5],
            },
        }
    }

    output_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with output_yaml_path.open('w', encoding='utf-8') as f:
        yaml.safe_dump(result, f, sort_keys=False)

    if not args.no_debug_plot:
        save_debug_plot(z_error, residuals, args.ransac_distance_threshold, debug_plot_path)

    if not args.no_view_plot:
        view_points = filter_points(
            read_pcd_xyz(Path(used_files[0])),
            args.z_min_filter,
            args.z_max_filter,
            args.max_points_per_frame,
            rng,
        )
        save_applied_scan_view(
            points_lidar=view_points,
            normal=normal,
            d=d,
            threshold=args.ransac_distance_threshold,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            translation=np.array([final_x, final_y, final_z], dtype=np.float64),
            ground_z=args.ground_z,
            validation=validation,
            yaml_path=output_yaml_path,
            pcd_path=Path(used_files[0]),
            path=view_plot_path,
            max_points=args.view_max_points,
            rng=rng,
        )

        if not args.no_first_person_plot:
            save_lidar_first_person_view(
                points_lidar=view_points,
                normal=normal,
                d=d,
                threshold=args.ransac_distance_threshold,
                pcd_path=Path(used_files[0]),
                path=first_person_plot_path,
                max_points=args.view_max_points,
                rng=rng,
            )
    elif not args.no_first_person_plot:
        view_points = filter_points(
            read_pcd_xyz(Path(used_files[0])),
            args.z_min_filter,
            args.z_max_filter,
            args.max_points_per_frame,
            rng,
        )
        save_lidar_first_person_view(
            points_lidar=view_points,
            normal=normal,
            d=d,
            threshold=args.ransac_distance_threshold,
            pcd_path=Path(used_files[0]),
            path=first_person_plot_path,
            max_points=args.view_max_points,
            rng=rng,
        )

    print('')
    print('========== LiDAR Ground Calibration Result ==========')
    print(f'estimated_roll_deg:     {math.degrees(roll): .6f}')
    print(f'estimated_pitch_deg:    {math.degrees(pitch): .6f}')
    print(f'manual_yaw_deg:         {args.manual_yaw_deg: .6f}')
    print(f'estimated_height:       {height: .6f}')
    print(f'final_x:                {final_x: .6f}  (manual)')
    print(f'final_y:                {final_y: .6f}  (manual)')
    print(f'final_z:                {final_z: .6f}  (estimated)')
    print(f'final_roll_rad:         {roll: .9f}')
    print(f'final_pitch_rad:        {pitch: .9f}')
    print(f'final_yaw_rad:          {yaw: .9f}  (manual)')
    print(f'used_frames:            {len(planes)}')
    print(f'inlier_count_median:    {inlier_count_median: .1f}')
    print(f'mean_z_error:           {validation["mean_z_error"]: .9f}')
    print(f'median_z_error:         {validation["median_z_error"]: .9f}')
    print(f'max_abs_z_error:        {validation["max_abs_z_error"]: .9f}')
    print(f'std_z_error:            {validation["std_z_error"]: .9f}')
    print(f'output_yaml_path:       {output_yaml_path}')
    print(f'debug_plot_path:        {debug_plot_path if not args.no_debug_plot else "disabled"}')
    print(f'view_plot_path:         {view_plot_path if not args.no_view_plot else "disabled"}')
    print(f'first_person_plot_path: {first_person_plot_path if not args.no_first_person_plot else "disabled"}')
    print('=====================================================')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
