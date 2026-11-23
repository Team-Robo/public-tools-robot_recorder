"""Which topics go in a robot-view bag.

Everything is a regex on the topic name, so topics that appear after recording
started (a node launched later) are picked up too. Raw images are excluded
because image_compressor republishes them as JPEG on <topic>/compressed.
"""

# Always recorded.
GROUPS = {
    'camera': [
        # Colour streams (recorded as /compressed), camera_info, and camera tf helpers
        r'^/camera/.*',
    ],
    'perception': [
        r'^/perception/.*',                # people, faces, poses, seats, lidar, debug images
    ],
    'hri': [
        r'^/speech/.*',                    # segments (audio of each utterance), transcripts
        r'^/audio/doorbell.*',
        r'^/robot_face/.*',
        r'^/conversation_llm/.*',
        r'^/behaviour_tree/.*',            # mission status, active leaf
        r'^/core/scheduler/status$',
        r'^/rosout$',                      # node logs, handy when replaying
    ],
    'nav': [
        r'^/tf$', r'^/tf_static$',
        r'^/map.*', r'.*costmap.*',                 # /map, /local_costmap/costmap, /global_costmap/*
        r'^/scan.*',                                # /scan, /scan_filtered (pointcloud_to_laserscan)
        r'^/odom.*', r'^/amcl_pose$', r'^/pose$',
        r'^/cmd_vel.*', r'^/motion_target/.*',      # base velocity / motion commands
        r'^/hdas/feedback_(chassis|torso).*',       # base/torso feedback
        r'.*plan$',                                 # /plan, /local_plan
        r'^/goal_pose$', r'.*/_action/status$',     # nav goals, navigate_to_pose status
        r'.*marker.*', r'^/semantic_boxes.*',       # semantic map, /align/markers
        r'^/robot_description$', r'^/joint_states$',
        r'^/lidar_filter/.*', r'^/battery_state$',
    ],
}

# Opt-in, by flag name: these are big or rarely needed.
OPTIONAL = {
    'depth': [r'.*depth.*'],               # raw 16-bit depth, ~18-25 MB/s per stream before zstd
    'audio': [r'^/audio/chunks$'],         # continuous mic stream, ~64 KB/s
    'lidar_raw': [r'^/hdas/lidar.*', r'^/livox/.*'],  # point clouds, several MB/s each
                                           # (/perception/lidar/points_nav is in perception)
}

# Never recorded.
ALWAYS_EXCLUDE = [
    r'^/parameter_events$',
    r'.*/_action/feedback$',               # action internals; results/status still come through
]

# Raw image topic names. Excluded unless --raw-images; their JPEG versions
# (<topic>/compressed) are what end up in the bag.
RAW_IMAGE = [
    r'.*/image_raw$', r'.*/image_rect_raw$', r'.*/image_rect$', r'.*/image_color$', r'.*/image$',
]


def load_config(config_path):
    """Load optional YAML overrides for groups, optional, always_exclude, and raw_image."""
    import os
    import yaml

    with open(os.path.expanduser(config_path), 'r') as f:
        data = yaml.safe_load(f) or {}
    return {
        'groups': data.get('groups', GROUPS),
        'optional': data.get('optional', OPTIONAL),
        'always_exclude': data.get('always_exclude', ALWAYS_EXCLUDE),
        'raw_image': data.get('raw_image', RAW_IMAGE),
    }


def union(patterns):
    return '|'.join(f'(?:{p})' for p in patterns) if patterns else '(?!)'


def build(depth=False, audio=False, lidar_raw=False, raw_images=False, extra=(), config_path=None):
    """(include_regex, exclude_regex) for ros2 bag record -e / -x."""
    cfg = load_config(config_path) if config_path else {
        'groups': GROUPS,
        'optional': OPTIONAL,
        'always_exclude': ALWAYS_EXCLUDE,
        'raw_image': RAW_IMAGE,
    }
    active_groups = cfg['groups']
    active_optional = cfg['optional']
    active_raw_image = cfg['raw_image']

    include = [p for ps in active_groups.values() for p in ps] + list(extra)
    exclude = list(cfg['always_exclude'])
    for name, on in (('depth', depth), ('audio', audio), ('lidar_raw', lidar_raw)):
        patterns = active_optional.get(name, [])
        if on:
            include += patterns
        else:
            exclude += patterns
    if not raw_images:
        # depth is raw by nature; when --depth is on keep its raw images
        raw = active_raw_image if not depth else [f'^(?!.*depth){p}' for p in active_raw_image]
        exclude += raw
    return union(include), union(exclude)
