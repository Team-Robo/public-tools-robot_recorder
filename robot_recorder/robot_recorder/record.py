"""Record a "robot view" bag: camera (as JPEG), every perception output, speech/BT
state, and the map/costmap/TF side, for replaying later (Foxglove, RViz, video).

  ros2 run robot_recorder record                    # until Ctrl-C, into ~/bags/
  ros2 run robot_recorder record --duration 120 --name receptionist_try3
  ros2 run robot_recorder record --depth --audio    # also raw depth and the mic stream
  ros2 run robot_recorder record --list             # what would be recorded right now

Run it on the machine with the camera (jetson_main), so images are compressed
before they cross the LAN.
"""
import argparse
import datetime
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from . import groups
from .image_compressor import ImageCompressor

def _default_workspace_src():
    for var in ('ROS2_WS_SRC', 'ROBOHOME_WS_SRC'):
        val = os.environ.get(var)
        if val:
            return os.path.expanduser(val)
    cur = os.path.dirname(os.path.realpath(__file__))
    for _ in range(8):
        if os.path.basename(cur) in ('src', 'install', 'build'):
            ws_src = os.path.join(os.path.dirname(cur), 'src')
            if os.path.isdir(ws_src):
                return ws_src
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return os.path.expanduser('~/ros2_ws/src')


WORKSPACE_SRC = _default_workspace_src()


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', default='', help='bag folder name (default robotview_<host>_<time>)')
    ap.add_argument('--out', default='~/bags', help='parent folder for bags (default ~/bags)')
    ap.add_argument('--duration', type=float, default=0, help='stop after N seconds (default: until Ctrl-C)')
    ap.add_argument('--depth', action='store_true', help='also record raw depth images (big)')
    ap.add_argument('--audio', action='store_true', help='also record the continuous mic stream /audio/chunks')
    ap.add_argument('--lidar-raw', action='store_true', help='also record unfiltered lidar point clouds')
    ap.add_argument('--raw-images', action='store_true', help='record raw colour images too (big)')
    ap.add_argument('--no-compress', action='store_true', help="don't run the JPEG republisher")
    ap.add_argument('--jpeg-quality', type=int, default=80)
    ap.add_argument('--image-fps', type=float, default=10.0, help='max rate per compressed image stream')
    ap.add_argument('--zstd', action='store_true',
                    help='zstd-compress a sqlite3 bag (smaller, but Foxglove can no longer open it; mcap is '
                         'always compressed and stays Foxglove-readable)')
    ap.add_argument('--split', type=float, default=300, help='start a new bag file every N seconds (default 300)')
    ap.add_argument('--extra', action='append', default=[], metavar='REGEX', help='also record topics matching REGEX')
    ap.add_argument('--config', default='', help='optional YAML file overriding topic groups')
    ap.add_argument('--list', action='store_true', help='show which current topics would be recorded, then exit')
    return ap.parse_args(argv)


def available_storage():
    """'mcap' if the plugin is installed (smaller, Foxglove-native), else 'sqlite3'."""
    try:
        out = subprocess.run(['ros2', 'bag', 'record', '--help'], capture_output=True, text=True, timeout=20).stdout
    except (OSError, subprocess.TimeoutExpired):
        return 'sqlite3'
    return 'mcap' if re.search(r'\{[^}]*\bmcap\b', out) else 'sqlite3'


def git_heads():
    heads = {}
    if not os.path.isdir(WORKSPACE_SRC):
        return heads
    for name in sorted(os.listdir(WORKSPACE_SRC)):
        path = os.path.join(WORKSPACE_SRC, name)
        if not os.path.isdir(os.path.join(path, '.git')):
            continue
        try:
            sha = subprocess.run(['git', '-C', path, 'rev-parse', '--short', 'HEAD'],
                                 capture_output=True, text=True, timeout=5).stdout.strip()
            branch = subprocess.run(['git', '-C', path, 'branch', '--show-current'],
                                    capture_output=True, text=True, timeout=5).stdout.strip()
            dirty = subprocess.run(['git', '-C', path, 'status', '--porcelain', '--untracked-files=no'],
                                   capture_output=True, text=True, timeout=5).stdout.strip()
            heads[name] = f'{branch or "detached"}@{sha}' + (' (uncommitted changes)' if dirty else '')
        except (OSError, subprocess.TimeoutExpired):
            pass
    return heads


def write_info(path, args, storage, include, exclude):
    lines = [
        f'recorded_by: robot_recorder',
        f'host: {socket.gethostname()}',
        f'started: {datetime.datetime.now().isoformat(timespec="seconds")}',
        f'ros_domain_id: {os.environ.get("ROS_DOMAIN_ID", "0")}',
        f'storage: {storage}',
        f'options: {" ".join(sys.argv[1:]) or "(defaults)"}',
        f'include_regex: {include!r}',
        f'exclude_regex: {exclude!r}',
        'repos:',
    ] + [f'  {k}: {v}' for k, v in git_heads().items()]
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def list_topics(include, exclude):
    rclpy.init()
    node = rclpy.create_node('robot_recorder_list')
    time.sleep(2.0)  # let discovery fill in
    inc, exc = re.compile(include), re.compile(exclude)
    rows = sorted(node.get_topic_names_and_types())
    node.destroy_node()
    rclpy.shutdown()
    print(f'{"":2}{"topic":58} type')
    for name, types in rows:
        mark = '+' if inc.search(name) and not exc.search(name) else ('-' if inc.search(name) else ' ')
        print(f'{mark:2}{name:58} {", ".join(types)}')
    print('\n+ recorded   - matched but excluded (raw image, or an opt-in flag is off)   blank: not in any group')
    print('Raw colour images are recorded as <topic>/compressed while recording runs.')


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    include, exclude = groups.build(depth=args.depth, audio=args.audio, lidar_raw=args.lidar_raw,
                                    raw_images=args.raw_images, extra=args.extra,
                                    config_path=args.config or None)
    if args.list:
        list_topics(include, exclude)
        return

    name = args.name or f'robotview_{socket.gethostname()}_{datetime.datetime.now():%Y%m%d_%H%M%S}'
    parent = os.path.expanduser(args.out)
    os.makedirs(parent, exist_ok=True)
    bag_dir = os.path.join(parent, name)
    if os.path.exists(bag_dir):
        sys.exit(f'{bag_dir} already exists - pick another --name')

    storage = available_storage()
    cmd = ['ros2', 'bag', 'record', '-o', bag_dir, '-e', include, '-x', exclude,
           '-s', storage, '-d', str(int(args.split))]
    if storage == 'mcap':
        cmd += ['--storage-preset-profile', 'zstd_fast']  # chunk-compressed, still Foxglove-readable
    elif args.zstd:
        cmd += ['--compression-mode', 'file', '--compression-format', 'zstd']
    else:
        print('note: mcap storage not installed, writing uncompressed sqlite3 (images are JPEG already). '
              'For smaller bags: sudo apt install ros-humble-rosbag2-storage-mcap')

    # JPEG republisher in this process, so the bag can pick up <topic>/compressed
    compressor, executor, spin = None, None, None
    if not args.no_compress and not args.raw_images:
        active_groups = groups.load_config(args.config) if args.config else groups.GROUPS
        compress_patterns = active_groups.get('camera', []) + active_groups.get('perception', [])
        rclpy.init()
        compressor = ImageCompressor(groups.union(compress_patterns),
                                     quality=args.jpeg_quality, max_fps=args.image_fps)
        executor = SingleThreadedExecutor()
        executor.add_node(compressor)
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()

    print(f'recording -> {bag_dir}  (storage {storage}, new file every {int(args.split)} s)')
    print('Ctrl-C to stop' + (f', or it stops after {args.duration:.0f} s' if args.duration else ''))
    bag = subprocess.Popen(cmd, start_new_session=True)
    # the bag folder only exists once ros2 bag starts; write the info file next to it
    for _ in range(50):
        if os.path.isdir(bag_dir):
            break
        time.sleep(0.1)
    info_path = os.path.join(bag_dir if os.path.isdir(bag_dir) else parent, 'robot_view_info.yaml')
    write_info(info_path, args, storage, include, exclude)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    start = time.monotonic()
    while not stop.is_set() and bag.poll() is None:
        if args.duration and time.monotonic() - start >= args.duration:
            break
        stop.wait(0.5)

    if bag.poll() is None:
        print('\nstopping (finishing and compressing the last file)...')
        os.killpg(bag.pid, signal.SIGINT)
        try:
            bag.wait(120)
        except subprocess.TimeoutExpired:
            os.killpg(bag.pid, signal.SIGKILL)
    if compressor is not None:
        counts = {k: v for k, v in compressor.counts.items() if v}
        executor.shutdown()
        compressor.destroy_node()
        rclpy.shutdown()
        if counts:
            print('compressed frames: ' + ', '.join(f'{k} {v}' for k, v in counts.items()))
    if bag.returncode not in (0, -2, 130, None):
        print(f'ros2 bag record exited with {bag.returncode}', file=sys.stderr)
    if os.path.isdir(bag_dir):
        subprocess.run(['ros2', 'bag', 'info', bag_dir])
        print(f'\nreplay:  ros2 run robot_recorder play {bag_dir}')


if __name__ == '__main__':
    main()
