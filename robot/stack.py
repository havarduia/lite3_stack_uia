#!/usr/bin/env python3
"""stack - the robot with this project's additions: lidar odometry, Nav2, floor
maps and named places, and his voice. Built on the SDK's Lite3 (lite3_sdk_uia).

    source ~/robot/env/lite3_env.sh
    python3 -c "
    from robot.stack import StackRobot
    with StackRobot() as bot:
        bot.stand()
        bot.goto(1.5)
    "

From the shell: python3 -m robot.stack (no command lists them).
"""
import math
import time

from lite3 import ACTIONS, Lite3
from lite3.robot import RobotNode, ignoring_sigint
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray

from . import locate
from .maps import FIT_POSE_STD, POSE_STD, Maps
from .nav import LETHAL, Nav, status_text
from .voice import Voice

# Odometry comes through robot/lio_relay.py (docs/lidar.md), which can die on its
# own while the bridge still takes velocities; the SDK stops on stale odometry.
ODOM_TOPIC = 'odom_fused'


class StackNode(RobotNode):
    """The SDK's node, plus Nav2's costmaps and robot/locate.py."""

    @property
    def grid(self):
        """Nav2's global costmap, decoded from the newest bytes the first time it is asked for."""
        raw = self._grid_raw
        if raw is None:
            return None
        if self._grid[0] is not raw:
            self._grid = (raw, deserialize_message(raw, OccupancyGrid))
        return self._grid[1]

    @grid.setter
    def grid(self, value):
        """Only None: nav_stop() forgets the costmap."""
        if value is not None:
            raise ValueError('grid comes from /global_costmap/costmap; only None can be set')
        self._grid_raw, self._grid = None, (None, None)

    def __init__(self, odom_topic=ODOM_TOPIC):
        super().__init__(odom_topic)
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._grid_raw, self._grid = None, (None, None)
        self.local_grid = None      # Nav2's local costmap: a turn on the spot checks it (nav.turn_blocked)
        # Map mode: what robot/locate.py last said, when, and the odometry of that moment.
        self.loc, self.loc_time, self.loc_odom = None, 0.0, None
        # Kept as bytes, decoded on use (grid): a floor like the arcade's takes 0.43 s to
        # decode on the Xavier, every 2 s, on the thread that also reads /localized.
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda b: setattr(self, '_grid_raw', b), latched, raw=True)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: setattr(self, 'local_grid', m), latched)
        self.create_subscription(Float64MultiArray, locate.TOPIC, self._loc_cb, 1)

    def _loc_cb(self, m):
        if len(m.data) == len(locate.Reading._fields):
            self.loc, self.loc_time, self.loc_odom = locate.Reading(*m.data), time.monotonic(), self.odom


class StackRobot(Maps, Nav, Voice, Lite3):
    """Live handle on the robot, with Nav2, maps and voice. Use as a context manager."""
    node_class = StackNode
    odom_topic = ODOM_TOPIC

    def __init__(self, **kw):
        kw.setdefault('odom_topic', ODOM_TOPIC)
        super().__init__(**kw)

    # Ctrl-C alone is not a stop once Nav2 drives: controller_server is another
    # process publishing its own cmd_vel.
    def estop(self, disarm=False):
        """Stop as hard as software can: ignore SIGINT meanwhile, cancel the Nav2
        goal, halt, kill Nav2 (it would keep publishing), then the SDK's estop
        (cancel, halt again, and with disarm=True drop the heartbeat)."""
        with ignoring_sigint():
            try:
                self.goto_cancel()
            except Exception:
                pass
            self._goal_handle = None        # goto() stops waiting: Nav2 is about to die
            self.halt()
            try:
                self.nav_stop()
            except Exception:
                pass
            return super().estop(disarm)

    def destroy_clients(self):
        for name in ('_nav_client', '_plan_client'):
            client = getattr(self, name)
            if client is not None:
                try:
                    client.destroy()
                except Exception:
                    pass
                setattr(self, name, None)

    def status(self):
        return (super().status() + ' nav2=' + ('up' if self.nav_running else 'down')
                + self._floor_status())

    def _floor_status(self):
        floor = self.floor
        if floor is None:
            return ''
        m = self.map_pose
        return ' floor=%s %s' % (floor, self.lost_why() or 'localized') + (
            ' at (%.2f, %.2f, %.0f deg)' % (m[0], m[1], math.degrees(m[2])) if m else '')


def _cli():
    import sys
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        print('commands: status stand sit auto walk <m> turn <deg> scan '
              'nav-start nav-stop goto <m> cost estop [--disarm] action <%s>\n'
              'on a floor map: floors, load <floor>, here <x> <y> <deg> [fit], places, '
              'save <name> [x y deg], forget <name>, go <name>, go <x> <y> [deg]'
              % '|'.join(ACTIONS))
        return
    cmd = args[0]
    with StackRobot(auto_mode=(cmd not in ('status', 'scan', 'cost', 'floors', 'places', 'save', 'forget'))) as bot:
        if cmd == 'status':
            time.sleep(0.5)     # let the sonar and IMU topics arrive
            print(bot.status())
        elif cmd == 'stand':
            bot.stand(); print(bot.status())
        elif cmd == 'sit':
            bot.sit(); print(bot.status())
        elif cmd == 'auto':
            bot.auto(); print('auto mode set')
        elif cmd == 'walk':
            print(bot.walk(float(args[1])))
        elif cmd == 'turn':
            print(bot.turn_deg(float(args[1])))
        elif cmd == 'scan':
            print(bot.scan_text())
        elif cmd == 'nav-start':
            bot.nav_start(); print('nav2 up')
        elif cmd == 'nav-stop':
            bot.nav_stop(); print('nav2 down')
        elif cmd == 'goto':
            print('status', bot.goto(float(args[1])))
        elif cmd == 'estop':
            print('estop:', bot.estop(disarm='--disarm' in args))
            print(bot.status())
        elif cmd == 'action':
            bot.action(args[1]); print(bot.status())
        elif cmd == 'floors':
            print('\n'.join(bot.floors()) or 'no floors: fetch a map with env/get_map.sh')
        elif cmd == 'load':
            bot.load_floor(args[1]); print('floor %s loaded; now: here <x> <y> <deg>' % args[1])
        elif cmd == 'here':
            # 'fit' after the numbers: they are check_map.py's best fit, so AMCL is told they are good
            std = FIT_POSE_STD if args[4:5] == ['fit'] else POSE_STD
            bot.set_pose(*map(float, args[1:4]), std=std); time.sleep(0.5); print(bot.status())
        elif cmd == 'places':
            for p in bot.places():
                print('%-24s (%.2f, %.2f, %.0f deg)' % (p['name'], p['x'], p['y'], p['yaw_deg']))
        elif cmd == 'save':
            time.sleep(0.8)     # let robot/locate.py be heard
            print(bot.save_place(args[1], *map(float, args[2:5])))
        elif cmd == 'forget':
            bot.delete_place(args[1]); print('forgot', args[1])
        elif cmd == 'go':
            time.sleep(0.8)
            numbers = len(args) > 2
            status = bot.go_to_point(*map(float, args[1:4])) if numbers else bot.go_to(args[1])
            print(status_text(status), bot.status())
        elif cmd == 'cost':
            for d, c in bot.cost_ahead():
                mark = ' LETHAL' if c is not None and c >= LETHAL else (
                    ' inflated' if c is not None and c > 50 else '')
                print('  +%4.2f m  cost %s%s' % (d, c, mark))
        else:
            print('unknown command: ' + cmd)


if __name__ == '__main__':
    _cli()
