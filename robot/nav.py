"""nav - the Nav2 stack: start/stop it, read its costmap, goto().
Mixed into Lite3, so these are bot.nav_start(), bot.goto(1.5) etc.

Nav2 runs on a floor map only (maps.py: load_floor, places): the Orin's
Nav2 plans and drives, perception keeps the map and the costmaps
(env/start_nav2_map.sh). goto() goes some metres ahead of where he is.
"""
import math
import os
import signal
import subprocess
import time

from lite3 import Lite3Error
from lite3.motion import clamp, dist  # noqa: F401  (callers import them from here)

from . import places as place_file
from .lio_relay import wrap

# --- nav2 ---
# The stack runs in its own process group (setsid) and is killed by that
# group: killing by name misses its children, or reaches the
# static_transform_publisher the services use. docs/navigation.md.
NAV_MARKERS = ('start_nav2_map', 'robot.sonar_range', 'robot.orin_nav')
# Never kill a group holding one of these. Executables, not words: the
# launch line itself says launch_realsense:=false.
NAV_NEVER = ('jetson2motion', 'transfer_ros2', 'start_transfer',
             'realsense2_camera', 'realsense_ros2', 'voa_composition',
             'voa_ros2')

LETHAL = 99
LOCAL_STALE_S = 1.0     # the Orin's local costmap comes twice a second (robot/orin_nav.py)
SUCCEEDED, CANCELED, ABORTED = 4, 5, 6      # NavigateToPose result status, as goto() returns it
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the scripts in env/ move with the code


def status_text(status):
    """A goto() status in words."""
    return {SUCCEEDED: 'arrived', CANCELED: 'cancelled',
            ABORTED: 'aborted - no path'}.get(status, 'status %s' % status)


# A turn on the spot checks his body against the costmap first. His footprint
# in the Nav2 config is 0.6 x 0.4 m with 0.05 m padding: these are half of it.
# OBSTACLE is how Nav2 publishes a lethal cell (99 is "too close", not "on it").
FOOTPRINT_HALF = (0.35, 0.25)
OBSTACLE = 100
SWEEP_STEP = math.radians(5)


def cell_of(grid, x, y):
    """The (column, row) of (x, y), in the grid's frame, on a costmap whose origin may be turned (the
    Orin's local costmap, re-anchored on this odom by robot/orin_nav.py)."""
    o = grid.info.origin
    q = getattr(o, 'orientation', None)
    yaw = 2 * math.atan2(q.z, q.w) if q is not None else 0.0
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - o.position.x, y - o.position.y
    r = grid.info.resolution
    return int(math.floor((c * dx + s * dy) / r)), int(math.floor((-s * dx + c * dy) / r))


def cell(grid, x, y):
    """A costmap's value at (x, y) in its frame, or None off it."""
    i = grid.info
    cx, cy = cell_of(grid, x, y)
    if not (0 <= cx < i.width and 0 <= cy < i.height):
        return None
    return int(grid.data[cy * i.width + cx])


def under(grid, pose, half=FOOTPRINT_HALF):
    """Indices of the costmap cells his body covers at pose (x, y, yaw), in the grid's frame."""
    i = grid.info
    r, (hx, hy) = i.resolution, half
    c, s = math.cos(pose[2]), math.sin(pose[2])
    cells = set()
    nx, ny = int(2 * hx / (r / 2)) + 1, int(2 * hy / (r / 2)) + 1      # half a cell apart: no gaps
    for a in range(nx):
        for b in range(ny):
            bx, by = -hx + 2 * hx * a / (nx - 1), -hy + 2 * hy * b / (ny - 1)
            cx, cy = cell_of(grid, pose[0] + bx * c - by * s, pose[1] + bx * s + by * c)
            if 0 <= cx < i.width and 0 <= cy < i.height:
                cells.add(cy * i.width + cx)
    return cells


def sweep_hit(grid, pose, turn, half=FOOTPRINT_HALF, step=SWEEP_STEP):
    """The first part of `turn` (rad, signed) at which his body would cover an obstacle cell,
    or None if the whole turn is clear. Cells he covers already are left out: standing
    against something is not this turn's doing, and would refuse every turn."""
    there = under(grid, pose, half)
    n = max(1, int(math.ceil(abs(turn) / step)))
    for k in range(1, n + 1):
        part = turn * k / n
        if any(grid.data[j] >= OBSTACLE for j in under(grid, (pose[0], pose[1], pose[2] + part), half) - there):
            return part
    return None


def free_ahead(profile):
    """Metres a cost_ahead() profile stays plannable from the robot outwards:
    the last distance before the first LETHAL or off-map cell.
    """
    far = 0.0
    for d, c in profile:
        if c is None or c >= LETHAL:
            break
        far = d
    return far


def goal_ahead(profile, margin=0.25, minimum=0.5):
    """A goto() distance `margin` short of the first cell goto() would refuse,
    or None when that is under `minimum`: set when xy_goal_tolerance was 0.25
    (0.15 now); a goal inside the tolerance "arrives" at once."""
    goal = free_ahead(profile) - margin
    return goal if goal >= minimum else None


def _alive(pgid):
    """Is any process of that group still running?"""
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    # A dead leader stays on as a zombie until reaped; zombies do not count.
    ps = subprocess.run(['ps', '-eo', 'pgid=,stat='], capture_output=True, text=True)
    states = [f[1] for f in map(str.split, ps.stdout.splitlines()) if f[:1] == [str(pgid)]]
    return ps.returncode != 0 or any(not s.startswith('Z') for s in states)


def _kill_group(pgid, timeout, wait):
    """SIGINT, then SIGTERM, then SIGKILL, each given timeout/3 s to empty the group."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        if wait(lambda: not _alive(pgid), timeout / 3.0, 0.2):
            return


class Nav:
    """Needs _floor_grid(), self._node's local costmap, pose, turn() and halt(), from StackRobot."""
    _nav_client = None
    _goal_handle = None

    @property
    def nav_running(self):
        # robot.orin_nav: the Orin's Nav2 drives, and its goals come through that
        return subprocess.run(['pgrep', '-f', r'robot\.orin_nav'],
                              capture_output=True).returncode == 0

    @staticmethod
    def _nav_groups():
        """{pgid: [command lines]} of the groups that are a nav2 stack and hold no
        NAV_NEVER process.
        """
        out = subprocess.run(['ps', '-eo', 'pgid=,args='],
                             capture_output=True, text=True).stdout
        groups = {}
        for line in out.splitlines():
            line = line.strip()
            if not line:
                continue
            pgid, _, args = line.partition(' ')
            if not pgid.isdigit():
                continue
            # Drop launch arguments (foo:=bar) before matching, so a value
            # like launch_realsense:=false cannot look like a realsense node.
            args = ' '.join(t for t in args.split() if ':=' not in t)
            groups.setdefault(int(pgid), []).append(args)
        mine = os.getpgid(0)
        return {g: cmds for g, cmds in groups.items()
                if g != mine
                and any(m in c for c in cmds for m in NAV_MARKERS)
                and not any(n in c for c in cmds for n in NAV_NEVER)}

    def nav_stop(self, timeout=8.0):
        """Stop the nav2 stack by process group. Returns the number killed."""
        killed = 0
        for pgid in self._nav_groups():
            _kill_group(pgid, timeout, self._wait)
            killed += 1
        time.sleep(1.0)
        # after the sleep: a costmap the dead stack sent last could still land just after the kill
        self._node.local_grid = None
        return killed

    def nav_start(self, timeout=40.0, floor=None):
        """Launch Nav2 on a floor's map (load_floor() is the usual way in). Refuses
        unless the robot is standing and its pose has settled - a costmap built
        across the stand-up pose jump shows a clear path straight into a real
        obstacle."""
        if floor is None:
            raise Lite3Error('Nav2 runs on a floor map only: load a floor first (load_floor, or Load on the panel)')
        self._require_standing()
        if not self.pose_settled():
            raise Lite3Error(
                'pose is still moving; refusing to build a costmap around it. '
                'Wait for the robot to settle after standing.')
        place_file.floor_dir(floor)             # a floor that is there, before anything is stopped
        self.nav_stop()
        # setsid: a session and process group of its own, for nav_stop().
        with open('/tmp/nav2.log', 'w') as log:
            subprocess.Popen(['setsid', os.path.join(REPO, 'env', 'start_nav2_map.sh'), floor],
                             stdout=log, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL)
        if not self._wait(lambda: self.nav_running, timeout):
            raise Lite3Error('nav2 did not come up; see /tmp/nav2.log')
        # The Orin's Nav2 is up only once he is placed (set_pose waits for it)
        if not self._wait(lambda: self._loc() is not None, timeout):
            raise Lite3Error('nav2 is up but robot/locate.py says nothing; see /tmp/nav2.log')
        groups = self._nav_groups()
        if len(groups) > 1:
            raise Lite3Error(
                '%d nav2 process groups are running - an earlier stack '
                'survived. Call nav_stop() and retry.' % len(groups))
        return True

    def _local_grid(self):
        """The Orin's local costmap (camera and lidar, in odom), or None if it is not coming."""
        n = self._node
        g, at = getattr(n, 'local_grid', None), getattr(n, 'local_grid_at', None)
        return g if g is not None and (at is None or time.monotonic() - at < LOCAL_STALE_S) else None

    def turn_blocked(self, radians):
        """Why a turn on the spot by `radians` would put his body on an obstacle, or None.
        Checks the Orin's local costmap (camera and lidar), else the floor's map. With
        neither, says nothing: there is nothing to check."""
        local = self._local_grid()
        g = local or self._floor_grid()
        if g is None:
            return None
        frame = g.header.frame_id
        pose = self._node.odom if frame == 'odom' else self.map_pose if frame == 'map' else None
        if pose is None:
            return None
        hit = sweep_hit(g, pose, radians)
        if hit is None:
            return None
        return 'turning %+.0f deg would touch something %.0f deg in (%s costmap) - not turning' % (
            math.degrees(radians), math.degrees(abs(hit)), 'local' if g is local else 'floor map')

    def cost_at(self, x, y, frame='odom'):
        """Cost (0-100) at a point given in odom, or on the floor map (frame='map'): the Orin's
        local costmap where it covers the point (what the camera and lidar see now), else the
        floor's map (walls grown by half his width). None if off the map."""
        local = self._local_grid()
        if local is not None:
            ox, oy, _ = self._in_frame(x, y, 0.0, frame, 'odom') if frame != 'odom' else (x, y, 0.0)
            c = cell(local, ox, oy)
            if c is not None:
                return c
        g = self._floor_grid()
        if g is None:
            raise Lite3Error('no floor map - load a floor first')
        x, y, _ = self._in_nav_frame(x, y, 0.0, frame)
        return cell(g, x, y)

    def cost_ahead(self, out_to=3.5, step=0.25, settle=0):
        """[(distance, cost), ...] along the robot's heading. settle: re-read until
        two profiles a second apart agree, at most this many times. Use it after
        nav_start(): the costmap starts empty and unseen cells read as free.
        """
        def profile():
            x, y, yaw = self.wait_pose()
            return [(d, self.cost_at(x + d * math.cos(yaw),
                                     y + d * math.sin(yaw)))
                    for d in [i * step for i in range(int(out_to / step) + 1)]]

        prev = None
        for _ in range(settle):
            prof = profile()
            if prof == prev:
                return prof
            prev = prof
            time.sleep(1.0)
        return prev if prev is not None else profile()

    def goto(self, forward, heading_deg=None, timeout=60.0, check=True):
        """Navigate `forward` metres ahead, routing around obstacles. Returns the
        action status (SUCCEEDED, CANCELED or ABORTED).

        heading_deg: final heading relative to the one at the call (+ve left),
        done with turn() after arrival; Nav2 itself only gets him to the spot.
        Refuses a LETHAL goal cell, and a goal under 0.2 m (use turn_deg()).
        """
        self._require_standing()
        self._require_battery()
        if not self.nav_running:
            raise Lite3Error('Nav2 is not running - load a floor first')
        if abs(forward) < 0.2:
            raise Lite3Error(
                'goal is %.2f m away - too close for the planner to produce a '
                'path, it will abort. Use turn_deg() for rotation in place.'
                % abs(forward))
        x, y, yaw = self.wait_pose()
        gx, gy = x + forward * math.cos(yaw), y + forward * math.sin(yaw)
        gyaw = yaw + math.radians(heading_deg or 0.0)

        if check:
            c = self.cost_at(gx, gy)
            if c is None:
                raise Lite3Error('goal (%.2f, %.2f) is outside the costmap' % (gx, gy))
            if c >= LETHAL:
                raise Lite3Error(
                    'goal cell cost %d is LETHAL - NavFn cannot plan into it '
                    'and would abort with status 6. Pick a goal from '
                    'cost_ahead(), not from scan(): the costmap knows about '
                    'obstacles the camera cannot currently see.' % c)

        began = time.monotonic()
        handle = self.goal_send(gx, gy, gyaw)
        if self._stop_at > began:           # cancelled while Nav2 was taking the goal
            handle.cancel_goal_async()
        status = self._goal_wait(handle, timeout)
        if status == SUCCEEDED and heading_deg is not None:
            err = wrap(gyaw - self.wait_pose()[2])
            if abs(err) > math.radians(5):
                self.turn(err)
        return status

    def _goal_wait(self, handle, timeout, lost=None):
        """Wait out the goal just sent and return its action status. lost(): a
        reason to give up on the way, or None."""
        result_fut = self._goal_result
        why = [None]

        def over():
            why[0] = lost() if lost else None
            # estop() from another thread drops the handle: Nav2 is being killed and
            # the result may never come.
            return result_fut.done() or self._goal_handle is None or why[0]
        try:
            if not self._wait(over, timeout):
                handle.cancel_goal_async()
                self.halt()
                raise Lite3Error('goal timed out after %.0fs; cancelled' % timeout)
            if why[0] and not result_fut.done():
                handle.cancel_goal_async()
                self.halt()
                raise Lite3Error('lost on the way, stopped: ' + why[0])
        finally:
            self._goal_handle = None
        status = result_fut.result().status if result_fut.done() else CANCELED
        if status != SUCCEEDED:
            self.halt()                     # whoever ended it, he ends stopped
        return status

    def goal_send(self, gx, gy, gyaw=0.0, frame='odom'):
        """Hand Nav2 a goal at a point given in odom (or frame='map') and return
        once it is accepted, not when he arrives. A goal sent while another is
        under way replaces it. goto_cancel() and estop() end it. No checks:
        goto() and go_to_point() are the ones that refuse a bad goal."""
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped
        from rclpy.action import ActionClient

        if self._nav_client is None:        # one for the life of the node, not one per goal
            self._nav_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        client = self._nav_client
        if not client.wait_for_server(timeout_sec=10.0):
            raise Lite3Error('navigate_to_pose action server not available')

        # The Orin's Nav2 takes goals on the map only: an odom point is moved onto it here.
        nx, ny, nyaw = self._in_nav_frame(gx, gy, gyaw, frame)
        if frame != 'odom':
            gx, gy, gyaw = self._in_frame(gx, gy, gyaw, frame, 'odom')
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(nx), float(ny)
        goal.pose.orientation.z = math.sin(nyaw / 2.0)
        goal.pose.orientation.w = math.cos(nyaw / 2.0)
        msg = NavigateToPose.Goal()
        msg.pose = goal

        fut = client.send_goal_async(msg)
        if not self._wait(fut.done, 10.0):
            raise Lite3Error('no response to the goal')
        handle = fut.result()
        if handle is None or not handle.accepted:
            raise Lite3Error('goal rejected')
        # Published so estop() can cancel the goal even when the SIGINT
        # handler fires somewhere else entirely.
        self._goal_handle = handle
        self._goal_result = handle.get_result_async()
        self.goal = (gx, gy, gyaw)
        return handle

    goal = None     # (x, y, yaw) in odom of the goal last accepted; goal_active() says if it still stands

    def goal_active(self):
        """Is Nav2 still working on the goal last sent?"""
        return self._goal_handle is not None and not self._goal_result.done()

    def goto_cancel(self):
        """Ask Nav2 to drop the goal goto() is waiting on, from another
        thread; goto() then returns CANCELED. True if there was one. Nav2
        itself stays up - estop() is the one that kills it."""
        self.cancel()                        # ends a turn or a walk too, and a Go that has no goal out yet
        handle = self._goal_handle
        if handle is None:
            return False
        handle.cancel_goal_async()
        return True
