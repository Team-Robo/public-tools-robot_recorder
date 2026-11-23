"""JPEG-republish raw images so a bag holds ~0.5 MB/s per camera instead of ~9-27 MB/s.

For every sensor_msgs/Image topic that matches the camera/perception groups it
publishes sensor_msgs/CompressedImage on <topic>/compressed (image_transport's
naming, so Foxglove and RViz show it as that image). Topics are rediscovered
every couple of seconds, so cameras and debug images that start later are
covered. Skips a topic if something else already publishes its /compressed,
and skips depth (16-bit; record it raw with --depth instead).
"""
import re
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

IMAGE_TYPE = 'sensor_msgs/msg/Image'


class ImageCompressor(Node):

    def __init__(self, topic_regex: str, quality: int = 80, max_fps: float = 10.0):
        super().__init__('robot_recorder_image_compressor')
        self._pattern = re.compile(topic_regex)
        self._quality = int(quality)
        self._min_dt = 1.0 / max_fps if max_fps > 0 else 0.0
        self._subs = {}        # raw topic -> (subscription, publisher)
        self._last = {}        # raw topic -> last publish time (monotonic)
        self._lock = threading.Lock()
        self.counts = {}
        self.create_timer(2.0, self._discover)
        self._discover()

    def _discover(self):
        for name, types in self.get_topic_names_and_types():
            if IMAGE_TYPE not in types or name in self._subs:
                continue
            if 'depth' in name or not self._pattern.search(name):
                continue
            out = f'{name}/compressed'
            if any(p.node_name != self.get_name() for p in self.get_publishers_info_by_topic(out)):
                continue  # the camera driver already publishes it
            # Match the camera's reliability: a best-effort subscription to a
            # reliable publisher silently drops big frames (a ~1 MB image is
            # many UDP fragments; losing one loses the frame).
            reliable = any(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                           for p in self.get_publishers_info_by_topic(name))
            qos = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
            pub = self.create_publisher(CompressedImage, out, qos)
            sub = self.create_subscription(
                Image, name, lambda msg, n=name, p=pub: self._on_image(n, p, msg), qos)
            self._subs[name] = (sub, pub)
            self.counts[name] = 0
            self.get_logger().info(f'compressing {name} -> {out} ({"reliable" if reliable else "best effort"})')

    def _on_image(self, name, pub, msg: Image):
        now = time.monotonic()
        if now - self._last.get(name, 0.0) < self._min_dt:
            return
        self._last[name] = now
        img = _to_bgr(msg)
        if img is None:
            return
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header          # original capture stamp, so replay lines up
        out.format = 'jpeg'
        out.data = buf.tobytes()
        pub.publish(out)
        self.counts[name] += 1


def _to_bgr(msg: Image):
    enc = msg.encoding.lower()
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1, '8uc1': 1, '8uc3': 3}.get(enc)
    if channels is None:
        return None  # 16-bit/float images: not a JPEG job
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    row = msg.step if msg.step else msg.width * channels
    arr = arr.reshape(msg.height, row)[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if enc == 'rgb8':
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if enc == 'bgra8':
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    return arr


def main(args=None):
    """Standalone: ros2 run robot_recorder image_compressor"""
    from .groups import GROUPS, union
    rclpy.init(args=args)
    node = ImageCompressor(union(GROUPS['camera'] + GROUPS['perception']))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
