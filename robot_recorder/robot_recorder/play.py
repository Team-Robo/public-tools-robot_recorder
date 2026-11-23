"""Replay a robot-view bag.

  ros2 run robot_recorder play ~/bags/robotview_...            # once, real time
  ros2 run robot_recorder play ~/bags/robotview_... --loop --rate 0.5

Publishes /clock too, so start viewers with use_sim_time:=true if they care
about time (RViz, nodes). Foxglove can also open the bag folder's .db3/.mcap
files directly, no ROS needed.

Don't replay on the live robot's ROS domain while the real nodes run: the
replayed perception topics would mix with the live ones. Use another domain:
  ROS_DOMAIN_ID=42 ros2 run robot_recorder play ...
"""
import argparse
import os
import subprocess
import sys


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--loop', action='store_true')
    ap.add_argument('--start', type=float, default=0.0, help='skip the first N seconds')
    ap.add_argument('--topics', nargs='*', default=[], help='only these topics')
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    bag = os.path.expanduser(args.bag)
    if not os.path.isdir(bag):
        sys.exit(f'no bag folder at {bag}')
    cmd = ['ros2', 'bag', 'play', bag, '--clock', '-r', str(args.rate)]
    if args.loop:
        cmd.append('--loop')
    if args.start:
        cmd += ['--start-offset', str(args.start)]
    if args.topics:
        cmd += ['--topics', *args.topics]
    info = os.path.join(bag, 'robot_view_info.yaml')
    if os.path.isfile(info):
        print(open(info).read())
    sys.stdout.flush()  # exec replaces this process; unflushed output would be lost
    os.execvp(cmd[0], cmd)


if __name__ == '__main__':
    main()
