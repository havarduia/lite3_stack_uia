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

from lite3 import Lite3Error

from . import locate
from . import places as place_file
from .lio_relay import inv, join, wrap
from .nav import CANCELED, LETHAL, SUCCEEDED, dist

LOC_STALE_S = 3.0       # locate.py speaks twice a second; quiet this long, he does not know where he is.
                        # Was 1.5: on the 200 m arcade map the panel's ROS thread fell behind and six
                        # goals in a row ended "couldn't get there" (2026-10-15, cause not proven)
POSE_AGAIN_S = 8.0      # s before a pose not yet taken is said again: longer than the Orin's search (docs/navigation.md)
ROUTE_TIMEOUT = 300.0   # s for one go_to() at least: 30 m takes about two minutes
ROUTE_SLOWEST = 0.2     # m/s: longer routes get their length at this speed (135 m took over 300 s, 2026-10-15).
                        # A last resort only: Nav2's progress checker catches him stuck within 10 s.
THERE_M = 0.2           # nearer than this the planner makes no path, and he is there anyway
SHORT_M = 0.6           # "succeeded" further than this from the point is not believed (it stops 0.1 to 0.2 m off)
COSTMAP_WAIT_S = 60.0   # s after he is placed for Nav2's first global costmap (arcade floor: >30 s, 2026-11-09)
CLEAR_SETTLE_S = 2.5    # the global costmap is redrawn once a second; the map and a fresh scan are back by then
# Nav2's controller here has nothing that rewards turning towards the path: with
# the goal behind him it stood for 20 s, then span as a "recovery" (2026-10-09,
# 57 s for 2 m). So he faces along the route himself before Nav2 gets the goal.
FACE_OVER = math.radians(60)    # if its start is further off his heading than this
FACE_AT_M = 0.5                 # judged by the route's point this far along


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

    def _nav_frame(self):
        """The frame Nav2 plans in: its costmap says. odom until there is one."""
        g = self._node.grid
        return g.header.frame_id if g is not None else 'odom'

    def _in_nav_frame(self, x, y, yaw, frame):
        """A pose given in 'odom' or 'map', in the frame Nav2 plans in."""
        return self._in_frame(x, y, yaw, frame, self._nav_frame())

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
                # Placed. Goals still need the first global costmap (its frame and costs), and on
                # a big floor that comes long after the pose: wait for it on its own clock.
                if self._wait(lambda: self._node.grid is not None, COSTMAP_WAIT_S):
                    return True
                raise Lite3Error('placed at (%.2f, %.2f), but Nav2 sent no global costmap in %.0f s more; '
                                 'see /tmp/nav2.log' % (x, y, COSTMAP_WAIT_S))
            time.sleep(0.2)
        raise Lite3Error('gave the pose (%.2f, %.2f, %.0f deg), but: %s' % (
            x, y, yaw_deg, self.lost_why() or 'Nav2 did not finish starting; see /tmp/nav2.log'))

    _plan_client = None

    _clear_client = None

    def _forget_marks(self, timeout=5.0):
        """Empty Nav2's global costmap; the map and what the sensors see now
        come back at its next update. False if Nav2 did not do it."""
        from nav2_msgs.srv import ClearEntireCostmap

        if self._clear_client is None:
            self._clear_client = self._node.create_client(
                ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        if not self._clear_client.wait_for_service(timeout_sec=timeout):
            return False
        if not self._wait(self._clear_client.call_async(ClearEntireCostmap.Request()).done, timeout):
            return False
        time.sleep(CLEAR_SETTLE_S)
        return True

    def route_to(self, x, y, timeout=10.0, again=True):
        """The way Nav2's planner finds from where he is to a map point, as
        [(x, y), ...] on the map. Raises if there is none, having asked twice:
        the second time with the costmap's old marks forgotten."""
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient

        if self._plan_client is None:
            self._plan_client = ActionClient(self._node, ComputePathToPose, 'compute_path_to_pose')
        if not self._plan_client.wait_for_server(timeout_sec=timeout):
            raise Lite3Error('the planner is not answering - is nav2 running?')
        msg = ComputePathToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.w = 1.0
        msg.planner_id = 'GridBased'
        sent = self._plan_client.send_goal_async(msg)
        if not self._wait(sent.done, timeout) or sent.result() is None or not sent.result().accepted:
            raise Lite3Error('the planner did not take the question')
        answer = sent.result().get_result_async()
        if not self._wait(answer.done, timeout):
            raise Lite3Error('the planner did not answer in %.0f s' % timeout)
        poses = answer.result().result.path.poses if answer.result().status == SUCCEEDED else []
        if not poses:
            # a mark beyond the lidar's clearing range outlives what made it: a door seen shut stays shut
            if again and self._forget_marks():
                return self.route_to(x, y, timeout, again=False)
            raise Lite3Error('no route to (%.2f, %.2f): something is in the way, or it is cut off from here' % (x, y))
        return [(p.pose.position.x, p.pose.position.y) for p in poses]

    def go_to_point(self, x, y, yaw_deg=None, timeout=ROUTE_TIMEOUT, say=None):
        """Navigate to a point on the floor map. Returns the action status, as
        goto() does; goto_cancel() and estop() end it the same way.

        yaw_deg: the heading to end on, in the map's frame; None leaves him
        facing the way he arrived. Refuses unless he knows where he is, and a
        point off the map, in a wall or off the mapped floor, and one the
        planner finds no way to. Stops, and raises, if he stops knowing where
        he is on the way. say(what): told 'start' once he sets off, for the
        panel to speak.
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
            route = self.route_to(x, y)
            if self._stop_at > began:       # cancelled while the planner was asked
                return CANCELED
            say('start')
            ahead = next((p for p in route if dist(here, p) >= FACE_AT_M), route[-1])
            off = wrap(math.atan2(ahead[1] - here[1], ahead[0] - here[0]) - here[2])
            if abs(off) > FACE_OVER:
                self.turn(off)
                if self._stop_at > began:   # cancelled in the turn: no goal goes out
                    return CANCELED
                why = self.lost_why()       # a turn takes seconds; he may not know where he is any more
                if why:
                    raise Lite3Error('lost on the way, stopped: ' + why)
            gyaw = math.atan2(y - here[1], x - here[0]) if yaw_deg is None else math.radians(yaw_deg)
            length = sum(dist(a, b) for a, b in zip(route, route[1:]))
            end = time.monotonic() + max(timeout, length / ROUTE_SLOWEST)
            for again in (True, False):
                handle = self.goal_send(x, y, gyaw, frame='map')
                if self._stop_at > began:   # cancelled while Nav2 was taking the goal
                    handle.cancel_goal_async()
                status = self._goal_wait(handle, max(0.0, end - time.monotonic()), lost=self.lost_why)
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
