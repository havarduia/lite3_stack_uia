#!/usr/bin/env python3
"""check_motion.py - what the motion code promises, checked with NO robot.

    python3 ~/robot/demos/check_motion.py     # needs rclpy to import, nothing else

A fake node, a fake clock and a pretend Nav2 stand in for the robot, and no
packet or ROS message leaves this process, so it is safe to run anywhere -
next to a live robot too. Run it after touching robot/stack.py, nav.py or
maps.py. The SDK's own checks are in lite3_sdk_uia/tests/.
"""
import math
import os
import subprocess
import sys
import tempfile
import time
import types

import rclpy.action
from builtin_interfaces.msg import Time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lite3 import Lite3, Lite3Error, motion as M, testing as T  # noqa: E402
from robot import locate, maps, nav, places  # noqa: E402
from robot.stack import ODOM_TOPIC, StackNode, StackRobot  # noqa: E402


class World(T.World):
    """The SDK's fake robot, plus what the stack listens to: the floor's map, the
    Orin's local costmap and word on its Nav2, and what robot/locate.py last said
    (with the odometry of that moment). None of it until a floor is loaded and he
    is placed (on_floor())."""
    grid = loc = loc_stopped = None
    local_grid = None           # the Orin's local costmap, in odom: what a turn on the spot checks first
    local_grid_at = None        # when it came (the fake clock); None: just now
    nav_up = True               # the Orin says its Nav2 is up
    refused = (None, -1e9)
    ready_at = property(lambda self: time.monotonic() if self.nav_up else -1e9)
    loc_odom = (0.0, 0.0, 0.0)
    loc_time = property(lambda self: self.t if self.loc_stopped is None else self.loc_stopped)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=Time))

    def create_publisher(self, kind, topic, depth):     # /initialpose
        return types.SimpleNamespace(publish=POSES.append)


class Bot(StackRobot):
    nav_running = True                      # no pgrep: the pretend Nav2 below is always up
    floor = 'lab'                           # nor for the floor that is loaded
    _grid_floor = 'lab'                     # its map is the World's grid, not read from a file


def run(move, bot=Bot, **setup):
    """move(bot) on a fresh fake robot -> (its result or exception, was he left
    halted, every Twist sent)."""
    return T.run(move, bot, World, **setup)


GOALS, CANCELS = [], []      # every goal pose the pretend Nav2 was sent, and when one was cancelled
NO_ROUTE = []                # put anything in it: the pretend Orin refuses goals, no way there
POSES = []                   # every pose he was told he is at
EARLY = []                   # one entry per goal the pretend Nav2 calls succeeded at once, unmoved


def pretend_nav2(arrives):
    """rclpy's ActionClient, pretended. A goal ends when it is cancelled, or
    `arrives` seconds after it was sent, or (None) never. Returns the list of
    clients made."""
    made = []
    del GOALS[:], CANCELS[:]

    def client(node, action, name):
        goal = {}
        def done():
            if goal['refused']:
                node.refused = ('no route', time.monotonic())   # robot/orin_nav.py says why, beside the abort
                return True
            if goal['cancelled'] or goal['early']:
                return True
            if arrives is None or node.t < goal['sent'] + arrives:
                return False
            if node.loc is not None:        # map mode: arrived means he stands on the point
                to = goal['pose'].pose.position
                node.loc, node.loc_odom = node.loc._replace(x=to.x, y=to.y, yaw=node.bot.map_pose[2]), node.odom
            return True
        result = types.SimpleNamespace(
            done=done,
            result=lambda: types.SimpleNamespace(status=nav.ABORTED if goal['refused'] else
                                                 nav.CANCELED if goal['cancelled'] else nav.SUCCEEDED))
        handle = types.SimpleNamespace(accepted=True, get_result_async=lambda: result,
                                       cancel_goal_async=lambda: CANCELS.append(node.t) or goal.update(cancelled=True))
        made.append(name)
        return types.SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda msg: GOALS.append(msg.pose) or goal.update(
                sent=node.t, cancelled=False, pose=msg.pose, early=bool(EARLY and EARLY.pop()),
                refused=bool(NO_ROUTE)) or types.SimpleNamespace(
                done=lambda: True, result=lambda: handle))

    rclpy.action.ActionClient = client
    sys.modules['nav2_msgs'] = sys.modules['nav2_msgs.action'] = types.SimpleNamespace(
        NavigateToPose=types.SimpleNamespace(Goal=types.SimpleNamespace))
    return made


def boom():
    raise ZeroDivisionError


def floor_costmap():
    """The floor's map as a costmap (maps.floor_grid): 10 x 10 m in the map frame,
    free but for a wall along x = 8 m."""
    cells = [0] * 10000
    for row in range(100):
        cells[row * 100 + 80] = 100
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id='map'), data=cells,
        info=types.SimpleNamespace(resolution=0.1, width=100, height=100, origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0))))


def grid_with(frame, obstacles, origin=(-2.0, -2.0), size=4.0, res=0.05):
    """A costmap `size` m square in `frame`, free but for an obstacle cell (100,
    as Nav2 publishes lethal) at each (x, y) in `obstacles`."""
    n = int(round(size / res))
    cells = [0] * (n * n)
    for x, y in obstacles:
        cells[int((y - origin[1]) / res) * n + int((x - origin[0]) / res)] = 100
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=frame), data=cells,
        info=types.SimpleNamespace(resolution=res, width=n, height=n, origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=origin[0], y=origin[1]))))


def on_floor(state=locate.LOCALIZED):
    """World settings for map mode: odometry reads (0, 0, 0) and locate.py has
    him at (5, 5) on the map, facing +y."""
    return {'grid': floor_costmap(), 'loc': locate.Reading(state, 5.0, 5.0, math.pi / 2, 0.1, 0.05, 0.95)}


def sent_goal():
    """(frame, x, y, yaw in degrees) of the goal the pretend Nav2 got last."""
    g = GOALS[-1]
    return (g.header.frame_id, round(g.pose.position.x, 3), round(g.pose.position.y, 3),
            round(math.degrees(2 * math.atan2(g.pose.orientation.z, g.pose.orientation.w))))


def check_stack():
    """What StackRobot adds to the SDK's Lite3."""
    assert StackRobot.node_class is StackNode and ODOM_TOPIC == 'odom_fused'
    seen, real = {}, Lite3.__init__
    Lite3.__init__ = lambda self, **kw: seen.update(kw)
    try:
        StackRobot()
    finally:
        Lite3.__init__ = real
    assert seen.get('odom_topic') == 'odom_fused', seen          # lidar odometry, not the SDK's leg odometry

    # the stack's estop also stops Nav2: it is another process publishing its own velocities
    stopped = []

    class Stopping(Bot):
        def nav_stop(self, timeout=8.0):
            stopped.append(timeout)
    r, halted, _ = run(lambda b: b.estop(), Stopping)
    assert r is True and stopped and halted, (r, stopped)
    del stopped[:]

    def ctrl_c(b):
        try:
            b._on_sigint(2, None)
        except KeyboardInterrupt:
            return 'interrupted'
    r, halted, _ = run(ctrl_c, Stopping)
    assert r == 'interrupted' and stopped and halted, (r, stopped)    # Ctrl-C reaches the stack's estop

    # close() asks destroy_clients() for the Nav2 action clients, while the node still lives
    gone = []
    bot = Bot.__new__(Bot)
    bot._nav_client = types.SimpleNamespace(destroy=lambda: gone.append('nav'))
    bot.destroy_clients()
    assert gone == ['nav'] and bot._nav_client is None, gone


def check_floor_grid(folder):
    """maps.floor_grid on a small picture: 10 x 6 cells of 0.1 m, a wall down x = 0.5 m, grey (never
    mapped) in the top row, free elsewhere."""
    w, h = 10, 6
    px = bytearray([254] * (w * h))
    for row in range(h):
        px[row * w + 5] = 0                     # the wall
    for col in range(w):
        px[col] = 205                           # the top row is the map's far edge: y = 0.5 to 0.6 m
    with open(os.path.join(folder, 'map.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h) + bytes(px))
    with open(os.path.join(folder, 'map.yaml'), 'w') as f:
        f.write('image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.15\n')   # as get_map.sh's maps
    g = maps.floor_grid(os.path.join(folder, 'map.yaml'))
    at = lambda x, y: nav.cell(g, x, y)        # noqa: E731
    assert g.header.frame_id == 'map' and (g.info.width, g.info.height) == (w, h)
    assert at(0.55, 0.2) == 100 and at(0.35, 0.2) == 100 and at(0.25, 0.2) == 0, (at(0.35, 0.2), at(0.25, 0.2))
    assert at(0.15, 0.55) == 100 and at(0.15, 0.2) == 0 and at(1.5, 0.2) is None      # grey is a wall; off it, None


def main():
    check_stack()
    # every way out of a motion call leaves him stopped, and says why
    r, halted, _ = run(lambda b: b.walk(0.5, stop_distance=None))
    assert r['reason'] == 'target reached' and 0.5 <= r['moved'] < 0.55 and halted, r
    r, halted, _ = run(lambda b: b.turn_deg(270, guard=False))   # past 180: counted, not wrapped
    assert r['reason'] == 'target reached' and 260 < r['turned_deg'] < 275 and halted, r
    # a turn on the spot looks at the costmap for his body first (2026-10-14: he turned 130 deg into
    # a parked robot's tyres): something it would swing into refuses the turn, and nothing is sent
    beside = grid_with('odom', [(0.0, -0.35)])          # 0.35 m to his right: outside him, inside his sweep
    r, halted, sent = run(lambda b: b.turn_deg(-130), local_grid=beside)
    assert 'would touch' in r['reason'] and r['turned'] == 0.0 and not any(c[2] for c in sent), (r, sent)
    r, _, _ = run(lambda b: b.turn_deg(130), local_grid=beside)       # the other way sweeps it too
    assert 'would touch' in r['reason'], r
    r, _, _ = run(lambda b: b.turn_deg(-130), local_grid=grid_with('odom', [(1.0, -1.0)]))
    assert r['reason'] == 'target reached', r                         # out of reach: turns
    r, _, _ = run(lambda b: b.turn_deg(-130), local_grid=grid_with('odom', [(0.2, 0.1)]))
    assert r['reason'] == 'target reached', r                         # under him already: not his turn's doing
    r, _, _ = run(lambda b: b.turn_deg(-130), local_grid=beside, local_grid_at=-10.0)
    assert r['reason'] == 'target reached', r                         # the Orin's costmap stopped coming: not believed
    assert nav.sweep_hit(beside, (0.0, 0.0, 0.0), math.radians(-20)) is None      # a small turn misses it
    assert nav.sweep_hit(beside, (0.0, 0.0, 0.0), math.radians(-90)) is not None
    r, halted, sent = run(lambda b: b.steer(lambda: (9.0, -9.0), limit=1.0))
    assert r['reason'] == 'time limit' and halted, r
    assert sent[0] == (M.MAX_SPEED, 0.0, -M.MAX_YAW_RATE), sent[0]
    r, halted, _ = run(lambda b: b.steer(lambda: (0.1, 0.0), limit=999))    # HARD_TIMEOUT caps it
    assert r['reason'] == 'time limit' and r['moved'] < 0.1 * (M.HARD_TIMEOUT + 1) and halted, r
    r, halted, _ = run(lambda b: b.steer(lambda: 'seen enough'))
    assert r['reason'] == 'seen enough' and halted, r
    r, halted, _ = run(lambda b: b.steer(boom))                  # a crash still ends stopped
    assert isinstance(r, ZeroDivisionError) and halted, r
    r, halted, _ = run(lambda b: b.steer(lambda: (0.2, 0.0)), at=(2.0, lambda w: setattr(w, 'stick_time', w.t)))
    assert r['reason'].startswith('handheld took over') and halted, r
    r, _, sent = run(lambda b: b.walk(0.5, stop_distance=None), state={'basic': 1, 'battery': 80})
    assert isinstance(r, Lite3Error) and not sent, r           # lying: refused, nothing sent
    r, _, sent = run(lambda b: b.steer(lambda: (float('nan'), float('nan')), limit=1.0))
    assert set(sent) == {(0.0, 0.0, 0.0)}, sent[0]               # a NaN is a stop, not full speed
    for bad in (lambda b: b.strafe(0.5, speed=0), lambda b: b.strafe(0.5, speed=9), lambda b: b.turn(1.0, rate=0)):
        r, _, sent = run(bad)
        assert isinstance(r, Lite3Error) and not sent, r       # refused, not a ZeroDivisionError

    # the odometry relay dies mid-walk: he stops, he does not walk on blind
    r, halted, _ = run(lambda b: b.walk(5.0, speed=0.5, stop_distance=None),
                       at=(1.0, lambda w: setattr(w, 'odom_stopped', w.t)))
    assert r['reason'].startswith('odometry stopped') and halted and r['moved'] < 1.5, r
    r, _, sent = run(lambda b: b.walk(1.0, stop_distance=None), odom_stopped=-10.0)
    assert isinstance(r, Lite3Error) and not sent, r           # and does not set off without it

    # a depth guard reads the cloud at 4 Hz however often the loop asks, and still stops him
    reads = []
    r, halted, _ = run(lambda b: setattr(b, 'clearance', lambda: reads.append(b._node.t) or 3.0 - b._node.odom[0])
                       or b.walk(5.0, speed=0.5, stop_distance=0.6))
    assert r['reason'].startswith('obstacle at') and 2.2 < r['moved'] < 2.6 and halted, r
    assert all(b - a >= 0.25 - 1e-9 for a, b in zip(reads[1:], reads[2:])), reads    # reads[0] is the pre-flight
    # a camera that stopped sending: its last picture is not "clear", he does not set off on it
    r, _, sent = run(lambda b: b.walk(1.0), cloud=object(), cloud_time=-5.0)
    assert isinstance(r, Lite3Error) and 'point cloud' in str(r) and not sent, r
    # Cancel (and so E-STOP) from another thread ends any motion, not only a Nav2 goal
    r, halted, _ = run(lambda b: b.walk(5.0, stop_distance=None), at=(1.0, lambda w: w.bot.goto_cancel()))
    assert r['reason'].startswith('cancelled') and halted and r['moved'] < 0.5, r

    # upright(): armed BEFORE waiting for ready, and sat down on the way out even after a crash
    calls = []
    for step in ('heartbeat_start', 'stand', 'sit'):
        setattr(Bot, step, lambda self, step=step: calls.append(step))
    Bot.wait_ready = lambda self: calls.append('wait_ready') or True

    def crash(b):
        with b.upright():
            boom()
    r, _, _ = run(crash)
    assert isinstance(r, ZeroDivisionError) and calls == ['heartbeat_start', 'wait_ready', 'stand', 'sit'], calls

    # Nav2 runs on a floor map only
    r, _, _ = run(lambda b: b.nav_start())
    assert isinstance(r, Lite3Error) and 'floor' in str(r), r

    # a Go: one action client however many goals, and three ways it ends
    made = pretend_nav2(arrives=2.0)
    r, _, _ = run(lambda b: [b.goto(1.0, check=False) for _ in range(3)], **on_floor())
    assert r == [nav.SUCCEEDED] * 3 and len(made) == 1, (r, made)
    pretend_nav2(arrives=None)                                   # a Nav2 that never gets there
    r, halted, _ = run(lambda b: b.goto(1.0, check=False), at=(1.0, lambda w: w.bot.goto_cancel()), **on_floor())
    assert r == nav.CANCELED and halted, r                       # cancelled from another thread
    r, halted, _ = run(lambda b: b.goto(1.0, check=False), at=(1.0, lambda w: setattr(w.bot, '_goal_handle', None)),
                       **on_floor())
    assert r == nav.CANCELED and halted, r                       # estop() dropped it: no waiting on a dead Nav2
    r, halted, _ = run(lambda b: b.goto(1.0, check=False, timeout=5.0), **on_floor())
    assert isinstance(r, Lite3Error) and 'timed out' in str(r) and halted, r
    r, _, sent = run(lambda b: b.goto(1.0, check=False))
    assert isinstance(r, Lite3Error) and not sent, r             # not placed: the Orin takes goals on the map only

    # --- map mode: Nav2 plans in the map frame, and so every goal must be in it ---
    pretend_nav2(arrives=2.0)
    r, _, _ = run(lambda b: b.goto(1.0, check=False), **on_floor())
    assert r == nav.SUCCEEDED and sent_goal() == ('map', 5.0, 6.0, 90), sent_goal()     # 1 m ahead of where he is on the map
    r, _, _ = run(lambda b: (b.cost_at(0.0, -3.05), b.cost_at(0.0, 0.0), b.cost_at(0.0, 9.0)), **on_floor())
    assert r == (100, 0, None), r           # odom points, looked up on the map: the wall at (8.05, 5), him, and (-4, 5) off it
    r, _, _ = run(lambda b: (b.cost_at(8.05, 5.0, frame='map'), b.cost_at(2.0, 5.0, frame='map')), **on_floor())
    assert r == (100, 0), r
    # where the Orin's local costmap reaches, it says (what the camera and lidar see now); stale, the map does
    r, _, _ = run(lambda b: (b.cost_at(0.0, -0.35), b.cost_at(5.375, 5.0, frame='map'), b.cost_at(0.0, -3.05)),
                  local_grid=beside, **on_floor())
    assert r == (100, 100, 100), r          # him at (0, 0) in odom is (5, 5) on the map; the wall is off the local one
    r, _, _ = run(lambda b: b.cost_at(0.0, -0.35), local_grid=beside, local_grid_at=-10.0, **on_floor())
    assert r == 0, r
    r, _, _ = run(lambda b: (b.localized, b.map_pose), **on_floor())
    assert r[0] is True and all(abs(a - b) < 1e-9 for a, b in zip(r[1], (5.0, 5.0, math.pi / 2))), r
    # he has walked 1 m since locate.py last spoke: the map pose has gone with him
    r, _, _ = run(lambda b: b.map_pose, odom=(1.0, 0.0, 0.0), **on_floor())
    assert all(abs(a - b) < 1e-9 for a, b in zip(r, (5.0, 6.0, math.pi / 2))), r
    r, _, _ = run(lambda b: (b.localized, b.map_pose))
    assert r == (False, None), r            # not placed: neither

    # go to a point on the map
    r, _, sent = run(lambda b: b.go_to_point(2.0, 5.0), **on_floor())
    assert r == nav.SUCCEEDED and sent_goal() == ('map', 2.0, 5.0, 180), sent_goal()  # facing the way he came
    # the way starts off to his left: no turn of his own first (that was for RPP); MPPI turns him as he goes
    assert not any(c[2] for c in sent), sent
    r, _, _ = run(lambda b: (b.go_to_point(2.0, 5.0, yaw_deg=0), b._node.odom[2]), **on_floor())
    assert r[0] == nav.SUCCEEDED and abs(r[1] + math.pi / 2) < 0.1, r                   # and turned to the heading asked for
    # the final turn to a heading looks too (global costmap in map mode): facing +y at (5, 5), turning to 0
    # would swing him over (5.35, 5.0); he arrives, says why he did not turn, and has not turned
    walled = dict(on_floor(), grid=grid_with('map', [(5.35, 5.0)], origin=(3.0, 3.0)))
    r, _, sent = run(lambda b: (b.go_to_point(5.05, 5.0, yaw_deg=0), b._node.odom[2]), **walled)
    assert r[0] == nav.SUCCEEDED and r[1] == 0.0 and not any(c[2] for c in sent), (r, sent)
    r, _, _ = run(lambda b: b.go_to_point(5.05, 5.0), **on_floor())
    assert r == nav.SUCCEEDED and len(GOALS) == 3, (r, len(GOALS))                      # already there: no goal sent
    pretend_nav2(arrives=None)
    r, halted, _ = run(lambda b: b.go_to_point(2.0, 5.0), at=(9.0, lambda w: w.bot.goto_cancel()), **on_floor())
    assert r == nav.CANCELED and halted, r

    def refused(move, **setup):
        before = len(GOALS)
        r, _, sent = run(move, **setup)
        assert isinstance(r, Lite3Error) and not sent and len(GOALS) == before, r     # no goal, no step
        return str(r)

    Bot.floor = None
    assert 'no floor' in refused(lambda b: b.go_to_point(2.0, 5.0))
    Bot.floor = 'lab'
    assert 'fit the map' in refused(lambda b: b.go_to_point(2.0, 5.0), **on_floor(locate.NO_FIT))
    assert 'not been told' in refused(lambda b: b.go_to_point(2.0, 5.0), **on_floor(locate.NO_POSE))
    assert 'says nothing' in refused(lambda b: b.go_to_point(2.0, 5.0), loc_stopped=-10.0, **on_floor())
    assert 'outside the map' in refused(lambda b: b.go_to_point(20.0, 5.0), **on_floor())
    assert 'wall' in refused(lambda b: b.go_to_point(8.05, 5.0), **on_floor())
    assert 'numbers' in refused(lambda b: b.go_to_point(float('nan'), 5.0), **on_floor())
    assert 'not standing' in refused(lambda b: b.go_to_point(2.0, 5.0), state={'basic': 1, 'battery': 80}, **on_floor())
    # cut off: the Orin refuses the goal at once (it has cleared its costmap and asked twice), and says why
    pretend_nav2(arrives=2.0)
    NO_ROUTE.append(True)
    r, halted, sent = run(lambda b: b.go_to_point(2.0, 5.0), **on_floor())
    assert isinstance(r, Lite3Error) and 'no route' in str(r) and halted and not any(any(c) for c in sent), r
    del NO_ROUTE[:]
    pretend_nav2(arrives=None)

    # Nav2 calls a goal succeeded at once, far from it (seen after a cancelled goal): sent again, believed only once there
    pretend_nav2(arrives=2.0)
    before = len(GOALS)
    EARLY.append(True)
    r, _, _ = run(lambda b: b.go_to_point(2.0, 5.0), **on_floor())
    assert r == nav.SUCCEEDED and len(GOALS) == before + 2, (r, len(GOALS) - before)
    EARLY.extend([True, True])
    r, _, _ = run(lambda b: b.go_to_point(2.0, 5.0), **on_floor())
    assert isinstance(r, Lite3Error) and 'arrived twice' in str(r), r
    pretend_nav2(arrives=None)

    # told where he is while a wide estimate happens to sit nearby (set down after a carry): the Orin must still hear it
    r, _, _ = run(lambda b: b.set_pose(5.0, 5.2, 90, timeout=2.0), **on_floor(locate.SPREAD))
    assert isinstance(r, Lite3Error) and 'too wide' in str(r) and len(POSES) == 1, (r, POSES)
    del POSES[:]

    # not placed yet: the pose is not said again while the Orin's search (a few seconds) may still be running
    r, _, _ = run(lambda b: b.set_pose(5.0, 5.0, 90, timeout=7.0), **on_floor(locate.NO_POSE))
    assert isinstance(r, Lite3Error) and len(POSES) == 1, (r, len(POSES))
    del POSES[:]
    r, _, _ = run(lambda b: b.set_pose(5.0, 5.0, 90, timeout=10.0), **on_floor(locate.NO_POSE))
    assert isinstance(r, Lite3Error) and len(POSES) == 2, (r, len(POSES))     # but is, in the end
    del POSES[:]

    # placed, but the Orin's Nav2 is late (a big floor's costmap): say so, not "Nav2 did not finish starting"
    late = maps.ORIN_READY_S
    maps.ORIN_READY_S = 0.3
    r, _, _ = run(lambda b: b.set_pose(5.0, 5.0, 90, timeout=2.0), nav_up=False, **on_floor())
    maps.ORIN_READY_S = late
    assert isinstance(r, Lite3Error) and str(r).startswith('placed at') and 'was not up' in str(r), r
    r, _, _ = run(lambda b: b.set_pose(5.0, 5.0, 90, timeout=2.0), **on_floor())
    assert r is True, r
    del POSES[:]

    # lost on the way: the goal is dropped and he stops, he does not walk on a wrong position
    r, halted, _ = run(lambda b: b.go_to_point(2.0, 5.0),
                       at=(9.0, lambda w: setattr(w, 'loc', w.loc._replace(state=locate.NO_FIT))), **on_floor())
    assert isinstance(r, Lite3Error) and 'lost' in str(r) and 'fit the map' in str(r) and halted and CANCELS, r
    del CANCELS[:]
    r, halted, _ = run(lambda b: b.go_to_point(2.0, 5.0), at=(9.0, lambda w: setattr(w, 'loc_stopped', w.t)), **on_floor())
    assert isinstance(r, Lite3Error) and 'lost' in str(r) and halted and CANCELS, r   # locate.py itself died

    # places: saved where he stands or at a point, gone to by name
    pretend_nav2(arrives=2.0)
    real_root = places.ROOT
    with tempfile.TemporaryDirectory() as places.ROOT:
        os.makedirs(os.path.join(places.ROOT, 'lab'))
        open(os.path.join(places.ROOT, 'lab', 'map.yaml'), 'w').close()

        def tour(b):
            here = b.save_place('Start')
            b.save_place('Window', 2.0, 5.0, 180)
            return here, b.go_to('window'), [p['name'] for p in b.places()], b.delete_place('START'), b.places()
        r, _, _ = run(tour, **on_floor())
        assert (r[0]['x'], r[0]['y'], r[0]['yaw_deg']) == (5.0, 5.0, 90.0), r
        assert r[1] == nav.SUCCEEDED and sent_goal() == ('map', 2.0, 5.0, 180) and r[2] == ['Start', 'Window'], r
        assert [p['name'] for p in r[4]] == ['Window'], r
        assert 'no place' in refused(lambda b: b.go_to('canteen'), **on_floor())
        assert 'fit the map' in refused(lambda b: b.save_place('Nowhere'), **on_floor(locate.NO_FIT))
        assert 'wall' in refused(lambda b: b.save_place('In the wall', 8.05, 5.0, 0), **on_floor())
        # Load lists only floors with a 3D map on the Orin (cloud.txt); 'lab' here has none
        os.makedirs(os.path.join(places.ROOT, 'hall'))
        for name in ('map.yaml', 'cloud.txt'):
            open(os.path.join(places.ROOT, 'hall', name), 'w').close()
        r, _, _ = run(lambda b: b.floors())
        assert r == ['hall'] and places.floors() == ['hall', 'lab'], r
        check_floor_grid(os.path.join(places.ROOT, 'hall'))
    places.ROOT = real_root

    # stopping a stack this process launched does not wait on its zombie (real time, real process)
    stack = subprocess.Popen(['setsid', 'sleep', '60'])
    assert StackRobot._wait(lambda: os.getpgid(stack.pid) == stack.pid, 2.0), 'setsid did not take'
    began = time.time()
    nav._kill_group(stack.pid, 6.0, StackRobot._wait)
    quick, gone = time.time() - began < 1.5, not nav._alive(stack.pid)
    stack.kill()                                                 # only matters if that failed
    assert quick and gone, 'nav_stop() would stall'
    print('check_motion ok')


if __name__ == '__main__':
    main()
