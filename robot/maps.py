"""maps - map mode: Nav2 on a floor map, where he is on it, named places.
Mixed into StackRobot, so these are bot.load_floor(), bot.go_to('window') etc.

A floor map under Nav2 (env/start_nav2_map.sh): the Orin's 3D localizer
(robot/cloud_locate.py) places him on it, robot/locate.py says whether to
believe it. docs/navigation.md.
"""
import math
import os
import subprocess
import time
from types import SimpleNamespace

import numpy as np
import yaml
from lite3 import Lite3Error

from . import locate
from . import places as place_file
from .lio_relay import inv, join, wrap
from .nav import ABORTED, LETHAL, SUCCEEDED, dist

LOC_STALE_S = 3.0       # locate.py speaks twice a second; quiet this long, he does not know where he is.
                        # Was 1.5: on the 200 m arcade map the panel's ROS thread fell behind and six
                        # goals in a row ended "couldn't get there" (2026-10-15, cause not proven)
POSE_AGAIN_S = 8.0      # s before a pose not yet taken is said again: longer than the Orin's search (docs/navigation.md)
ROUTE_TIMEOUT = 300.0   # s for one go_to() at least: 30 m takes about two minutes
ROUTE_SLOWEST = 0.2     # m/s: longer routes get their length at this speed (135 m took over 300 s, 2026-10-15).
                        # A last resort only: Nav2's progress checker catches him stuck within 10 s.
ROUTE_DETOUR = 2.0      # a route's length, from the straight line: the route itself is the Orin's
THERE_M = 0.2           # nearer than this the planner makes no path, and he is there anyway
SHORT_M = 0.6           # "succeeded" further than this from the point is not believed (it stops 0.1 to 0.2 m off)
ORIN_READY_S = 60.0     # s after he is placed for the Orin's Nav2 to be up (arcade floor: >30 s for a costmap, 2026-11-09)
READY_STALE_S = 2.0     # the Orin says it twice a second
REFUSED_WAIT_S = 0.5    # its word on a refused goal comes beside the abort, maybe just after it
FLOOR_RECHECK_S = 2.0   # which floor is loaded is asked of the running stack (pgrep) at most this often
WALL_GROW_M = 0.2       # half his width (the Orin's footprint): his middle stays this far off a wall


def floor_grid(yaml_path):
    """A floor map as a costmap in the map frame, as Nav2's map_server reads it: walls, grown by
    WALL_GROW_M, are lethal (100), and so is the grey never mapped (routes stay on what was mapped as
    free); the rest is 0."""
    with open(yaml_path) as f:
        info = yaml.safe_load(f)
    res = float(info['resolution'])
    x0, y0 = (float(v) for v in info['origin'][:2])
    picture = locate.read_pgm(os.path.join(os.path.dirname(yaml_path), info['image']))
    dark = picture / 255.0 if info.get('negate') else (255 - picture.astype(np.float32)) / 255.0
    dark = dark[::-1]           # the picture's top row is the map's far edge: row 0 here is y0
    wall = locate.grow((dark > float(info.get('occupied_thresh', 0.65))).copy(), int(round(WALL_GROW_M / res)))
    blocked = wall | ~(dark < float(info.get('free_thresh', 0.25)))
    h, w = blocked.shape
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map'), data=np.where(blocked, 100, 0).astype(np.int8).ravel(),
        info=SimpleNamespace(resolution=res, width=w, height=h,
                             origin=SimpleNamespace(position=SimpleNamespace(x=x0, y=y0))))


class Maps:
    """Needs Nav (goal_send, nav_start, turn_blocked) and the Lite3 core."""

    _pose_pub = None

    def floors(self):
        """The floors Nav2 can load: a map (~/lite3_maps/<floor>/, from env/get_map.sh) and a
        3D map on the Orin (cloud.txt). Older floors without one stay on disk, unlisted."""
        return [f for f in place_file.floors()
                if os.path.isfile(os.path.join(place_file.floor_dir(f), 'cloud.txt'))]

    @property
    def floor(self):
        """The floor Nav2 is running on, or None while Nav2 is down. Read off
        the running stack, so every Lite3 gives the same answer."""
        out = subprocess.run(['pgrep', '-af', r'robot\.locate '], capture_output=True, text=True).stdout
        words = out.split('\n')[0].split()       # ... robot.locate <floor>
        return words[words.index('robot.locate') + 1] if 'robot.locate' in words[:-1] else None

    def _floor(self):
        floor = self.floor
        if floor is None:
            raise Lite3Error('no floor is loaded - load_floor() first')
        return floor

    _grid_floor = None
    _floor_at = -1e9

    def _floor_grid(self):
        """The loaded floor's map as a costmap (floor_grid), read once per floor; None with none loaded."""
        now = time.monotonic()
        if now - self._floor_at > FLOOR_RECHECK_S:
            self._floor_at, floor = now, self.floor
            if floor != self._grid_floor:
                self._grid_floor, self._node.grid = floor, None
                if floor is not None:
                    self._node.grid = floor_grid(os.path.join(place_file.floor_dir(floor), 'map.yaml'))
        return self._node.grid

    @property
    def nav_ready(self):
        """The Orin's Nav2 is up and takes goals."""
        return time.monotonic() - getattr(self._node, 'ready_at', -1e9) < READY_STALE_S

    def load_floor(self, name, timeout=40.0):
        """Start Nav2 on that floor's map (replacing whatever Nav2 was up). He
        does not know where he is on it yet: set_pose() next, and Nav2 only
        finishes starting then."""
        return self.nav_start(timeout=timeout, floor=name)

    def _loc(self):
        """robot/locate.py's newest reading, or None once it has gone quiet."""
        n = self._node
        if n.loc is None or time.monotonic() - n.loc_time > LOC_STALE_S:
            return None
        return n.loc

    def lost_why(self):
        """None while he knows where he is on the map, else why not, in words."""
        loc = self._loc()
        if loc is None:
            return 'robot/locate.py says nothing (is a floor loaded?)'
        return None if int(loc.state) == locate.LOCALIZED else locate.WHY[int(loc.state)]

    @property
    def localized(self):
        return self.lost_why() is None

    def _fix(self):
        """odom -> map as (x, y, yaw): locate.py's last pose against the odometry
        of the moment it came. None while there is no pose."""
        loc, n = self._loc(), self._node
        if loc is None or int(loc.state) == locate.NO_POSE or n.loc_odom is None:
            return None
        return join((loc.x, loc.y, loc.yaw), inv(n.loc_odom))

    @property
    def map_pose(self):
        """(x, y, yaw) on the floor map, or None. Follows odometry between
        locate.py's two readings a second."""
        fix, odom = self._fix(), self._node.odom
        if fix is None or odom is None:
            return None
        x, y, yaw = join(fix, odom)
        return x, y, wrap(yaw)

    def _in_nav_frame(self, x, y, yaw, frame):
        """A pose given in 'odom' or 'map', on the map: Nav2 runs on a floor map only."""
        return self._in_frame(x, y, yaw, frame, 'map')

    def _in_frame(self, x, y, yaw, frame, target):
        """A pose given in 'odom' or 'map', in the other of the two."""
        if frame == target:
            return x, y, yaw
        fix = self._fix()
        if fix is None:
            raise Lite3Error('no way from %s to %s: %s' % (frame, target, self.lost_why() or 'no map pose'))
        return join(fix, (x, y, yaw)) if target == 'map' else join(inv(fix), (x, y, yaw))

    def set_pose(self, x, y, yaw_deg, timeout=30.0):
        """Tell him where he is on the loaded floor's map, roughly: within about
        half a metre and 20 degrees. Returns once robot/locate.py agrees and
        Nav2, which was waiting for this, has finished starting. Raises if what
        the lidar sees does not fit the map there."""
        from geometry_msgs.msg import PoseWithCovarianceStamped

        self._floor()
        self._require_standing()            # FAST-LIO2 starts once he stands, and the Orin places him with it
        if self._pose_pub is None:
            self._pose_pub = self._node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
        m.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
        end, said = time.monotonic() + timeout, None
        while time.monotonic() < end:
            loc = self._loc()
            taken = loc is not None and int(loc.state) != locate.NO_POSE and dist((loc.x, loc.y), (x, y)) < 1.0
            # Once whatever he believes (near and wide, after a carry, looks taken); then again
            # until the Orin has it: it may not be listening yet, or still hold an older pose.
            if said is None or (not taken and time.monotonic() - said > POSE_AGAIN_S):
                self._pose_pub.publish(m)
                said = time.monotonic()
            if taken and self.localized:
                # Placed. Goals still need the Orin's Nav2 up, and on a big floor that comes long
                # after the pose (its global costmap): wait for it on its own clock.
                if self._wait(lambda: self.nav_ready, ORIN_READY_S):
                    return True
                raise Lite3Error('placed at (%.2f, %.2f), but the Orin\'s Nav2 was not up %.0f s later; '
                                 'see /tmp/nav2.log' % (x, y, ORIN_READY_S))
            time.sleep(0.2)
        raise Lite3Error('gave the pose (%.2f, %.2f, %.0f deg), but: %s' % (
            x, y, yaw_deg, self.lost_why() or 'Nav2 did not finish starting; see /tmp/nav2.log'))

    def go_to_point(self, x, y, yaw_deg=None, timeout=ROUTE_TIMEOUT, say=None):
        """Navigate to a point on the floor map. Returns the action status, as
        goto() does; goto_cancel() and estop() end it the same way.

        yaw_deg: the heading to end on, in the map's frame; None leaves him
        facing the way he arrived. Refuses unless he knows where he is, and a
        point off the map, in a wall or off the mapped floor (floor_grid);
        raises when the Orin finds no way there (it refuses at once, with no
        recoveries). Stops, and raises, if he stops knowing where he is on the
        way. say(what): told 'start' once the goal is sent, for the panel to speak.
        """
        say = say or (lambda what: None)
        began = time.monotonic()
        for v in (x, y) if yaw_deg is None else (x, y, yaw_deg):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise Lite3Error('a map point is numbers, not %r' % (v,))
        self._require_standing()
        self._require_battery()
        self._floor()                       # refuses with no floor loaded
        why = self.lost_why()
        if why:
            raise Lite3Error('not going: ' + why)
        c = self.cost_at(x, y, frame='map')
        if c is None:
            raise Lite3Error('point (%.2f, %.2f) is outside the map' % (x, y))
        if c >= LETHAL:
            raise Lite3Error('point (%.2f, %.2f) is in a wall, an obstacle or off the mapped floor' % (x, y))
        here = self.map_pose
        status = SUCCEEDED
        if dist(here, (x, y)) >= THERE_M:
            # No turn to face the route first (that was for RPP, 2026-10-09): MPPI's PathAngleCritic
            # turns him along it as he sets off
            gyaw = math.atan2(y - here[1], x - here[0]) if yaw_deg is None else math.radians(yaw_deg)
            end = time.monotonic() + max(timeout, dist(here, (x, y)) * ROUTE_DETOUR / ROUTE_SLOWEST)
            for again in (True, False):
                sent = time.monotonic()
                handle = self.goal_send(x, y, gyaw, frame='map')
                if self._stop_at > began:   # cancelled while Nav2 was taking the goal
                    handle.cancel_goal_async()
                if again:
                    say('start')
                status = self._goal_wait(handle, max(0.0, end - time.monotonic()), lost=self.lost_why)
                if status == ABORTED and self._wait(lambda: self._node.refused[1] >= sent, REFUSED_WAIT_S):
                    raise Lite3Error('no route to (%.2f, %.2f): something is in the way, '
                                     'or it is cut off from here' % (x, y))
                short = dist(self.map_pose, (x, y))
                if status != SUCCEEDED or short < SHORT_M:
                    break
                # this Nav2 can answer "succeeded" at once to the goal that follows a cancelled one
                if not again:
                    raise Lite3Error('Nav2 said arrived twice, %.1f m short of (%.2f, %.2f)' % (short, x, y))
        if status == SUCCEEDED and yaw_deg is not None:
            err = wrap(math.radians(yaw_deg) - self.map_pose[2])
            if abs(err) > math.radians(5):
                r = self.turn(err)
                if r['reason'] != 'target reached':     # he is there; the heading is the lesser half
                    print('NOTE: arrived, but not turned to %.0f deg: %s' % (yaw_deg, r['reason']))
        return status

    def go_to(self, name, timeout=ROUTE_TIMEOUT):
        """Navigate to a saved place on the loaded floor, by name."""
        p = place_file.find(self._floor(), name)
        return self.go_to_point(p['x'], p['y'], p['yaw_deg'], timeout=timeout)

    def places(self):
        """The places saved on the loaded floor."""
        return place_file.load(self._floor())

    def save_place(self, name, x=None, y=None, yaw_deg=None):
        """Save a place on the loaded floor: where he stands and faces now, or
        a point on the map (heading 0 unless given)."""
        floor = self._floor()
        if x is None or y is None:
            why = self.lost_why()
            if why:
                raise Lite3Error('cannot save where he stands: ' + why)
            x, y, yaw = self.map_pose
            yaw_deg = math.degrees(yaw) if yaw_deg is None else yaw_deg
        else:
            c = self.cost_at(x, y, frame='map')
            if c is None or c >= LETHAL:
                raise Lite3Error('point (%.2f, %.2f) is off the map, in a wall or off the mapped floor' % (x, y))
        return place_file.save(floor, name, x, y, yaw_deg or 0.0)

    def rename_place(self, old, new):
        return place_file.rename(self._floor(), old, new)

    def delete_place(self, name):
        place_file.delete(self._floor(), name)
