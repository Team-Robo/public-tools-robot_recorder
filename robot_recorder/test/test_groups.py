import re

from robot_recorder.groups import build

def rec(name, **kw):
    inc, exc = build(**kw)
    return bool(re.search(inc, name)) and not re.search(exc, name)

def test_defaults():
    for t in ['/camera/color/image_raw/compressed', '/camera/color/camera_info', '/perception/people/poses',
              '/perception/debug/person_face/image/compressed', '/perception/seats', '/speech/transcripts',
              '/speech/segments', '/behaviour_tree/mission_status', '/core/scheduler/status', '/tf', '/tf_static',
              '/map', '/global_costmap/costmap', '/local_costmap/costmap_updates', '/scan', '/odom',
              '/perception/lidar/points_filtered', '/rosout', '/robot_face/state',
              # real names from nav-behaviour_tree / nav-semantic_map / robot_bringup
              '/camera/head_d455/color/image_raw/compressed', '/camera/left_d405/color/camera_info',
              '/perception/lidar/points_nav', '/scan_filtered', '/local_plan', '/goal_pose',
              '/navigate_to_pose/_action/status', '/motion_target/target_speed_chassis',
              '/hdas/feedback_chassis', '/semantic_boxes', '/align/markers', '/battery_state']:
        assert rec(t), t
    for t in ['/camera/color/image_raw', '/perception/debug/person_face/image', '/audio/chunks',
              '/camera/aligned_depth_to_color/image_raw', '/hdas/lidar_chassis_left', '/parameter_events',
              '/camera/head_d455/color/image_raw', '/camera/right_d405/depth/image_rect_raw',
              '/livox/lidar', '/livox/lidar_filtered', '/navigate_to_pose/_action/feedback',
              '/some/unrelated/topic']:
        assert not rec(t), t

def test_flags():
    assert rec('/camera/aligned_depth_to_color/image_raw', depth=True)
    assert not rec('/camera/color/image_raw', depth=True)      # colour raw still excluded
    assert rec('/audio/chunks', audio=True)
    assert rec('/hdas/lidar_chassis_left', lidar_raw=True)
    assert rec('/livox/lidar', lidar_raw=True)
    assert rec('/camera/right_d405/depth/image_rect_raw', depth=True)
    assert rec('/camera/color/image_raw', raw_images=True)
    assert rec('/my/thing', extra=[r'^/my/.*'])
