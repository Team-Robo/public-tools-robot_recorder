"""Turn a compressed image stream in a robot-view bag into an MP4.

  ros2 run robot_recorder export_video ~/bags/robotview_...                 # list image streams
  ros2 run robot_recorder export_video ~/bags/robotview_... /camera/color/image_raw/compressed
  ros2 run robot_recorder export_video BAG /perception/debug/person_face/image/compressed -o face.mp4

Frames are placed by their recorded time, so the video plays in real time
(gaps are filled by repeating the last frame). Debug-image topics already have
the perception overlays drawn in, which makes them the quickest "robot view".
"""
import argparse
import os
import sys

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

import rosbag2_py

COMPRESSED = 'sensor_msgs/msg/CompressedImage'


def open_reader(bag):
    meta = rosbag2_py.Info().read_metadata(bag, '')
    # bags recorded with --zstd need the decompressing reader
    reader = (rosbag2_py.SequentialCompressionReader() if meta.compression_format
              else rosbag2_py.SequentialReader())
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=meta.storage_identifier),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def image_topics(bag):
    meta = rosbag2_py.Info().read_metadata(bag, '')
    return [(t.topic_metadata.name, t.message_count) for t in meta.topics_with_message_count
            if t.topic_metadata.type == COMPRESSED]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('topic', nargs='?')
    ap.add_argument('-o', '--output', default='')
    ap.add_argument('--fps', type=float, default=15.0, help='output frame rate (default 15)')
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    bag = os.path.expanduser(args.bag)
    topics = image_topics(bag)
    if not args.topic:
        if not topics:
            sys.exit('no compressed image streams in this bag')
        print('compressed image streams:')
        for name, n in topics:
            print(f'  {name}  ({n} frames)')
        print('\npick one:  ros2 run robot_recorder export_video BAG <topic>')
        return
    if args.topic not in dict(topics):
        sys.exit(f'{args.topic} is not a compressed image stream in this bag')

    out = args.output or os.path.join(bag, args.topic.strip('/').replace('/', '_') + '.mp4')
    reader = open_reader(bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    writer, last, next_t, written = None, None, None, 0
    step = 1.0 / args.fps
    while reader.has_next():
        _topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, CompressedImage)
        frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 or t_ns * 1e-9
        if writer is None:
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (w, h))
            next_t = stamp
        if frame.shape[:2] != (h, w):  # a stream that changed size mid-bag
            frame = cv2.resize(frame, (w, h))
        while last is not None and next_t + step <= stamp:   # hold the previous frame over gaps
            writer.write(last)
            next_t += step
            written += 1
        if next_t <= stamp + step:
            writer.write(frame)
            next_t += step
            written += 1
        last = frame
    if writer is None:
        sys.exit('no decodable frames')
    writer.release()
    print(f'wrote {written} frames ({written / args.fps:.1f} s) -> {out}')


if __name__ == '__main__':
    main()
