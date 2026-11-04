"""Stream a numbered PCD sequence as PointCloud2 on ROS 2 Jazzy."""
import math
import re
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

DEFAULT_DIRECTORY = '/home/sjjin/pcd_cloud'
PCD_TYPES = {
    ('I', 1): ('i1', PointField.INT8), ('U', 1): ('u1', PointField.UINT8),
    ('I', 2): ('<i2', PointField.INT16), ('U', 2): ('<u2', PointField.UINT16),
    ('I', 4): ('<i4', PointField.INT32), ('U', 4): ('<u4', PointField.UINT32),
    ('F', 4): ('<f4', PointField.FLOAT32), ('F', 8): ('<f8', PointField.FLOAT64),
}


def natural_key(path):
    return [int(part) if part.isdigit() else part.lower()
            for part in re.split(r'(\d+)', path.name)]


def read_pcd(path):
    """Read one PCD; return a PointCloud2 without its header and stage timings."""
    start = time.perf_counter()
    with path.open('rb') as stream:
        header = {}
        for _ in range(64):
            line = stream.readline()
            if not line:
                raise ValueError(f'{path}: missing DATA header')
            decoded = line.decode('ascii').strip()
            if not decoded or decoded.startswith('#'):
                continue
            key, *values = decoded.split()
            header[key.upper()] = values
            if key.upper() == 'DATA':
                break
        else:
            raise ValueError(f'{path}: header too long')
        required = ('FIELDS', 'SIZE', 'TYPE', 'WIDTH', 'HEIGHT', 'POINTS', 'DATA')
        if any(key not in header for key in required):
            raise ValueError(f'{path}: missing PCD header entry')
        names = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        if not (len(names) == len(sizes) == len(types) == len(counts)):
            raise ValueError(f'{path}: inconsistent PCD field definitions')
        if len(set(names)) != len(names) or not {'x', 'y', 'z'} <= set(names):
            raise ValueError(f'{path}: duplicate fields or missing x/y/z')
        width, height, points = (int(header[key][0]) for key in ('WIDTH', 'HEIGHT', 'POINTS'))
        if min(width, height, points) < 0 or width * height != points:
            raise ValueError(f'{path}: WIDTH * HEIGHT differs from POINTS')
        fields, dtype_fields, offset = [], [], 0
        for name, size, kind, count in zip(names, sizes, types, counts):
            if count < 1 or (kind, size) not in PCD_TYPES:
                raise ValueError(f'{path}: unsupported {name} type {kind}{size} count {count}')
            numpy_type, ros_type = PCD_TYPES[(kind, size)]
            fields.append(PointField(name=name, offset=offset, datatype=ros_type, count=count))
            dtype_fields.append((name, numpy_type, (count,)) if count > 1 else (name, numpy_type))
            offset += size * count
        point_step = offset
        mode = header['DATA'][0].lower()
        if mode == 'binary':
            payload = stream.read(points * point_step + 1)
            if len(payload) != points * point_step:
                raise ValueError(f'{path}: binary payload size mismatch')
        elif mode == 'ascii':
            payload = stream.read()
        else:
            raise ValueError(f'{path}: DATA {mode} unsupported (expected binary or ascii)')
    read_end = time.perf_counter()
    dtype = np.dtype(dtype_fields, align=False)
    if mode == 'binary':
        data = payload
        array = np.frombuffer(data, dtype=dtype, count=points)
    else:
        values = np.fromstring(payload.decode('ascii'), sep=' ')
        expected = points * sum(counts)
        if len(values) != expected:
            raise ValueError(f'{path}: ASCII payload has {len(values)} values, expected {expected}')
        array = np.empty(points, dtype=dtype)
        cursor = 0
        matrix = values.reshape(points, sum(counts))
        for name, count in zip(names, counts):
            array[name] = matrix[:, cursor:cursor + count] if count > 1 else matrix[:, cursor]
            cursor += count
        data = array.tobytes()
    dense = all(np.isfinite(array[axis]).all() for axis in ('x', 'y', 'z'))
    parse_end = time.perf_counter()
    msg = PointCloud2()
    msg.height = height
    msg.width = width
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = width * point_step
    msg.data = data
    msg.is_dense = dense
    build_end = time.perf_counter()
    return msg, ((read_end - start) * 1000, (parse_end - read_end) * 1000,
                 (build_end - parse_end) * 1000)


def read_recorded_times(csv_path, files):
    """Read only the reliable numeric prefix and raw filename of each row.

    Some recordings have an unterminated quote in the final preview_path column.
    """
    stamps = {}
    with csv_path.open(encoding='utf-8') as stream:
        header = stream.readline().rstrip('\r\n').split(',')
        if header[:3] != ['frame_index', 'stamp_sec', 'stamp_nanosec']:
            raise ValueError(f'{csv_path}: missing timestamp prefix columns')
        for line in stream:
            parts = line.split(',', 3)
            if len(parts) < 4:
                raise ValueError(f'{csv_path}: incomplete row')
            sec, nsec = int(parts[1]), int(parts[2])
            if not 0 <= nsec < 1_000_000_000:
                raise ValueError(f'{csv_path}: invalid nanoseconds')
            match = re.search(r'raw/([^/",]+\.pcd)', parts[3])
            if not match or match.group(1) in stamps:
                raise ValueError(f'{csv_path}: missing or duplicate raw PCD path')
            stamps[match.group(1)] = sec + nsec * 1e-9
    if any(path.name not in stamps for path in files):
        raise ValueError(f'{csv_path}: missing timestamps for PCD files')
    ordered = [stamps[path.name] for path in files]
    if any(b <= a for a, b in zip(ordered, ordered[1:])):
        raise ValueError(f'{csv_path}: timestamps must increase')
    return ordered


class PcdSequencePlayer(Node):
    def __init__(self):
        super().__init__('pcd_sequence_player')
        defaults = dict(pcd_directory=DEFAULT_DIRECTORY, topic='/ch/lslidar_point_cloud',
                        frame_id='laser_link', fps=10.0, loop=True, start_frame=0,
                        end_frame=-1, timing_mode='fixed', paused=False)
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in defaults}
        directory = Path(p['pcd_directory']).expanduser()
        if not directory.is_dir():
            raise ValueError(f'PCD directory does not exist: {directory}')
        self.files = sorted(directory.glob('*.pcd'), key=natural_key)
        if not self.files:
            raise ValueError(f'No PCD files in {directory}')
        self.start = int(p['start_frame'])
        self.end = len(self.files) - 1 if int(p['end_frame']) == -1 else int(p['end_frame'])
        if not 0 <= self.start <= self.end < len(self.files):
            raise ValueError(f'Invalid zero-based range {self.start}..{self.end} for {len(self.files)} files')
        self.fps = float(p['fps'])
        if not math.isfinite(self.fps) or self.fps <= 0:
            raise ValueError('fps must be finite and positive')
        self.mode = str(p['timing_mode'])
        if self.mode not in ('fixed', 'recorded'):
            raise ValueError('timing_mode must be fixed or recorded')
        self.times = None
        if self.mode == 'recorded':
            csv_path = directory / 'frames.csv'
            if not csv_path.is_file():
                csv_path = directory.parent / 'frames.csv'
            self.times = read_recorded_times(csv_path, self.files)
        self.topic = str(p['topic'])
        self.frame_id = str(p['frame_id'])
        self.loop = bool(p['loop'])
        self.paused = bool(p['paused'])
        self.index = self.start
        self.published = 0
        self.done = False
        self.publisher = self.create_publisher(PointCloud2, self.topic, 10)
        self.reader = ThreadPoolExecutor(max_workers=2, thread_name_prefix='pcd_read')
        self.pending = {}
        self.prefetch()
        self.get_logger().info(
            f'Loaded PCD sequence: directory={directory}, frames={len(self.files)}, '
            f'first={self.files[0].name}, last={self.files[-1].name}; '
            f'zero-based range={self.start}..{self.end}, mode={self.mode}, '
            f'fps={self.fps}, loop={self.loop}, topic={self.topic}, frame_id={self.frame_id}')
        self.timer = self.create_timer(1.0 / self.fps if self.mode == 'fixed' else 0.001,
                                       self.publish_frame)

    def prefetch(self):
        indices = [self.index]
        if self.index < self.end:
            indices.append(self.index + 1)
        elif self.loop and self.start != self.end:
            indices.append(self.start)
        for index in indices:
            if index not in self.pending:
                self.pending[index] = self.reader.submit(read_pcd, self.files[index])

    def publish_frame(self):
        if self.paused:
            return
        frame_start = time.monotonic()
        if self.mode == 'recorded':
            self.timer.cancel()
        path = self.files[self.index]
        try:
            msg, durations = self.pending.pop(self.index).result()
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Cannot load frame at index {self.index}: {exc}')
            self.timer.cancel()
            self.done = True
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        publish_start = time.perf_counter()
        self.publisher.publish(msg)
        publish_ms = (time.perf_counter() - publish_start) * 1000
        self.published += 1
        if self.published == 1 or self.published % 50 == 0:
            self.get_logger().info(
                f'frame index={self.index}/{self.end} file={path.name} points={msg.width * msg.height} '
                f'read={durations[0]:.1f}ms parse={durations[1]:.1f}ms '
                f'build={durations[2]:.1f}ms publish={publish_ms:.1f}ms')
        previous = self.index
        if self.index == self.end:
            if not self.loop:
                self.timer.cancel()
                self.get_logger().info('Playback complete')
                self.done = True
                return
            self.index = self.start
            self.get_logger().info(f'Loop: {path.name} -> {self.files[self.start].name}')
        else:
            self.index += 1
        self.prefetch()
        if self.mode == 'recorded':
            if self.index > previous:
                delay = self.times[self.index] - self.times[previous]
            else:
                # Loop boundary has no recorded successor; use the target FPS.
                delay = 1.0 / self.fps
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(max(delay - (time.monotonic() - frame_start), 0.001),
                                           self.publish_frame)

    def destroy_node(self):
        self.reader.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PcdSequencePlayer()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (ValueError, OSError) as exc:
        print(f'PCD player error: {exc}')
        raise SystemExit(1) from exc
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
