#!/usr/bin/env python3
"""Python-side checks for jetson_snapshot.sh. Read-only.

  versions   which Python modules the pipeline needs are importable, and from where
  topics     grab one message from each pipeline topic, print what arrived, and save
             raw copies for offline analysis (run with the pipeline running):
               occupancy_grid.npz   the nvblox grid (int8 cell values) and its metadata
               camera_color.png     one colour frame
"""
import argparse
import importlib
import math
import os
import sys
import time

TOPICS = [
    ('occupancy', '/nvblox_node/static_occupancy_grid', 'nav_msgs.msg', 'OccupancyGrid'),
    ('pose', '/visual_slam/tracking/vo_pose', 'geometry_msgs.msg', 'PoseStamped'),
    ('color', '/camera0/color/image_raw', 'sensor_msgs.msg', 'Image'),
    ('heightmap', '/braille/heightmap', 'sensor_msgs.msg', 'Image'),
    ('braille_map', '/braille_map', 'std_msgs.msg', 'UInt8MultiArray'),
    ('braille_image', '/braille_image', 'std_msgs.msg', 'UInt8MultiArray'),
]
DOT_ROWS, DOT_COLS = 36, 80  # what the pipeline currently publishes


def cmd_versions(_args):
    for name in ('numpy', 'cv2', 'serial', 'crcmod', 'rclpy', 'cv_bridge', 'nvblox_msgs'):
        try:
            mod = importlib.import_module(name)
            version = getattr(mod, '__version__', getattr(mod, 'VERSION', '?'))
            print(f'{name:12s} {version:10s} {getattr(mod, "__file__", "")}')
        except Exception as e:  # noqa: BLE001 - report every failure, whatever it is
            print(f'{name:12s} IMPORT FAILED: {e!r}')


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def describe_grid(msg, np):
    info = msg.info
    vals = np.asarray(msg.data, dtype=np.int8)
    lines = [
        f'frame_id={msg.header.frame_id!r}  resolution={info.resolution:.4f} m/cell',
        f'size={info.width} x {info.height} cells '
        f'({info.width * info.resolution:.2f} x {info.height * info.resolution:.2f} m)',
        f'origin=({info.origin.position.x:.3f}, {info.origin.position.y:.3f}, '
        f'{info.origin.position.z:.3f})  origin yaw={yaw_deg(info.origin.orientation):.1f} deg',
    ]
    if vals.size == 0:
        return lines + ['(grid is empty)']
    uniq, counts = np.unique(vals, return_counts=True)
    lines.append('cell values (value: count):')
    lines += [f'  {int(u):4d}: {int(c):8d}  ({100.0 * c / vals.size:5.1f}%)' for u, c in zip(uniq, counts)]
    other = [int(u) for u in uniq if int(u) not in (-1, 0, 100)]
    if other:
        lines.append(f'NOTE: {len(other)} value(s) other than -1/0/100, e.g. {other[:10]}. '
                     'heightmap_generator_node only handles 0 and 100, so these are shown as unknown.')
    return lines


def save_grid(msg, np, path):
    info = msg.info
    np.savez_compressed(
        path,
        data=np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width),
        resolution=info.resolution,
        origin_xyz=np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z]),
        origin_yaw_deg=yaw_deg(info.origin.orientation),
        frame_id=msg.header.frame_id,
        stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
    )


def image_array(msg, np):
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        return None
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    arr = arr.reshape(msg.height, msg.step)[:, :msg.width * channels]
    arr = arr.reshape(msg.height, msg.width, channels) if channels == 3 else arr.reshape(msg.height, msg.width)
    return arr[:, :, ::-1] if msg.encoding == 'rgb8' else arr  # to BGR for cv2.imwrite


def ascii_dots(values, np):
    arr = np.asarray(values, dtype=np.uint8)
    if arr.size != DOT_ROWS * DOT_COLS:
        return [f'(size {arr.size}, expected {DOT_ROWS * DOT_COLS}; not drawn)']
    grid = arr.reshape(DOT_ROWS, DOT_COLS)
    return [''.join('#' if v else '.' for v in row) for row in grid]


def cmd_topics(args):
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data  # best effort: matches reliable publishers too

    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('braille_snapshot_probe')
    got = {}
    for key, topic, module, cls in TOPICS:
        try:
            msg_type = getattr(importlib.import_module(module), cls)
        except Exception as e:  # noqa: BLE001
            print(f'{topic}: cannot import {module}.{cls}: {e!r}')
            continue
        node.create_subscription(msg_type, topic, lambda m, k=key: got.setdefault(k, m), qos_profile_sensor_data)

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and len(got) < len(TOPICS):
        rclpy.spin_once(node, timeout_sec=0.2)

    for key, topic, _, _ in TOPICS:
        print(f'\n--- {topic}')
        msg = got.get(key)
        if msg is None:
            print(f'(nothing received in {args.timeout:.0f} s)')
            continue
        if key == 'occupancy':
            print('\n'.join(describe_grid(msg, np)))
            save_grid(msg, np, os.path.join(args.out, 'occupancy_grid.npz'))
            print('saved -> files/occupancy_grid.npz')
        elif key == 'pose':
            p = msg.pose.position
            print(f'frame_id={msg.header.frame_id!r}  position=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})  '
                  f'yaw={yaw_deg(msg.pose.orientation):.1f} deg')
        elif key in ('color', 'heightmap'):
            print(f'frame_id={msg.header.frame_id!r}  encoding={msg.encoding}  size={msg.width} x {msg.height}')
            arr = image_array(msg, np)
            if key == 'heightmap' and arr is not None:
                uniq, counts = np.unique(arr, return_counts=True)
                print('pixel values (value: count): ' + ', '.join(f'{int(u)}: {int(c)}' for u, c in zip(uniq, counts)))
            if key == 'color' and arr is not None:
                try:
                    import cv2
                    cv2.imwrite(os.path.join(args.out, 'camera_color.png'), arr)
                    print('saved -> files/camera_color.png')
                except Exception as e:  # noqa: BLE001
                    print(f'could not save image: {e!r}')
        else:
            vals = np.asarray(msg.data, dtype=np.uint8)
            print(f'{vals.size} values, distinct: {sorted(int(v) for v in np.unique(vals))}')
            print('\n'.join(ascii_dots(vals, np)))

    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('versions')
    t = sub.add_parser('topics')
    t.add_argument('--out', default='.', help='folder for saved files')
    t.add_argument('--timeout', type=float, default=20.0, help='seconds to wait for messages')
    args = ap.parse_args()
    return {'versions': cmd_versions, 'topics': cmd_topics}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
