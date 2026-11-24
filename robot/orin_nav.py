"""orin_nav - perception's end of the Orin's Nav2, which plans and drives.

Serves navigate_to_pose here, so robot/nav.py works as before, and passes each goal to the Orin
(orin/bin/orin_nav.py), where Nav2 Humble plans (Smac) and drives (MPPI, collision monitor). Back
come how the goal stands, the route (published here as /plan, for the panel) and the command, which
goes on /cmd_vel while the goal runs and is fresh (drive_cmd), then zeros, then quiet:
the panel's own turns and walks get through. The Orin cancels a goal when this goes quiet; this
ends one when the Orin does (ORIN_STALE_S).

The depth camera goes to the Orin as the obstacle voxels perception's local costmap keeps (STVL's
/local_costmap/voxel_grid, with its memory and frustum clearing), placed around base_link by the
odometry of that moment; the sonars as their ranges. The Orin sends no command while they are stale.
Perception keeps the map, the costmaps (the panel, cost_at, turn_blocked) and the planner for
route_to (env/nav2_map.launch.py), without bt_navigator.

Kept cheap: perception is ~75% busy without it. A tf listener (every /tf, 50+ a second, in Python)
cost ~0.4 core and starved the panel into "lost on the way" stops (big_walk, 2026-10-28).

    python3 -m robot.orin_nav [dry]     started by env/start_nav2_map.sh; dry: no /cmd_vel
    python3 -m robot.orin_nav check     self-check, no ROS
"""
import math
import signal
import socket
import struct
import sys
import threading
import time

from .lio_relay import ORIN, yaw_of

# The layouts and ports orin/bin/orin_nav.py reads (check() compares them)
ORIN_PORT = 8044
DRIVE_PORT = 8045       # MPPI's commands from the Orin
CMD = struct.Struct('<c3d')         # V, vx, vy, wz
DEPTH = struct.Struct('<cd3d')      # D, when, base_link x y yaw in odom; float32 (x, y) around base_link follow
SONAR = struct.Struct('<cB4f')      # S, 0 front 1 rear, range, min, max, field of view
SONARS = ('/sonar/front', '/sonar/rear')    # robot/sonar_range.py, 20 Hz
DEPTH_Z = (0.05, 0.7)   # m above the floor: STVL's lowest, and his height (nav.sh vehicleHeight)
MAX_VOXELS = 5000       # 40 kB: one datagram
DRIVE_VX = (-0.2, 0.5)  # m/s
DRIVE_WZ = 0.8          # rad/s, MPPI's own wz_max
MPPI_STALE_S = 0.3      # MPPI speaks at 10 Hz
DRIVE_HZ = 20
STOP_TICKS = 10         # zeros sent after driving ends (best effort drops some), then quiet

GOAL = struct.Struct('<cI3d')       # G, goal number, x y yaw on the map
CANCEL = struct.Struct('<cI')       # C, goal number
STATUS = struct.Struct('<cIB')      # A, goal number, action_msgs GoalStatus (0: not taken yet)
PATH = struct.Struct('<c')          # P; float32 (x, y) on the map follow
EXECUTING, SUCCEEDED, CANCELED, ABORTED = 2, 4, 5, 6
RESEND_S = 0.2
TAKE_S = 20.0           # the Orin takes a goal within this, or it is aborted here: its Nav2 needs
                        # some seconds after a start, and turns goals down till then
ORIN_STALE_S = 2.0      # it tells the goal's status twice a second
CANCEL_WAIT_S = 3.0
CAMERA_STALE_S = 1.0    # no depth frame for this long: the camera has stopped, no voxels go to the Orin


def _interrupt(*_):
    raise KeyboardInterrupt


def drive_cmd(mppi, mppi_age):
    """What to publish: MPPI's (vx, vy, wz) clamped, or zeros when MPPI has gone quiet."""
    if mppi is None or mppi_age > MPPI_STALE_S:
        return 0.0, 0.0, 0.0
    vx, _, wz = mppi
    return min(max(vx, DRIVE_VX[0]), DRIVE_VX[1]), 0.0, min(max(wz, -DRIVE_WZ), DRIVE_WZ)


def depth_packet(when, base, xyz):
    """Voxels in odom (n x 3) -> a datagram of those in DEPTH_Z, as (x, y) around base_link."""
    import numpy as np
    p = xyz[(xyz[:, 2] >= DEPTH_Z[0]) & (xyz[:, 2] <= DEPTH_Z[1])][:MAX_VOXELS]
    c, s = math.cos(base[2]), math.sin(base[2])
    dx, dy = p[:, 0] - base[0], p[:, 1] - base[1]
    rel = np.stack([c * dx + s * dy, -s * dx + c * dy], 1).astype(np.float32)
    return DEPTH.pack(b'D', when, *base) + rel.tobytes()


def goal_end(status, cancelling):
    """How a goal the Orin has ended ends here: 'succeed', 'canceled', 'abort', or None if it runs on."""
    if status == SUCCEEDED and not cancelling:
        return 'succeed'
    if status in (CANCELED, ABORTED, SUCCEEDED):
        return 'canceled' if cancelling else 'abort'
    return None


def main(dry=False):
    import numpy as np
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry, Path
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, PointCloud2, Range

    from lite3.robot import take_ready      # imports rclpy: not for check()
    from .stack import ODOM_TOPIC

    rclpy.init()
    node = rclpy.create_node('orin_nav')
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    to_orin = lambda data: out.sendto(data, (ORIN, ORIN_PORT))      # noqa: E731
    say = lambda *a: print('orin_nav:', *a, file=sys.stderr, flush=True)    # noqa: E731
    here = {}
    lock = threading.Lock()
    # the goal running: its number, and whether a cancel was asked; the Orin's word on each goal
    d = {'seq': int(time.time() * 10) & 0x7fffffff, 'active': None, 'cancelling': False, 'driving': False,
         'status': {}, 'mppi': None, 'at': 0.0, 'zeros': 0}

    # --- the camera and the sonars ---------------------------------------------------------
    odom_node = rclpy.create_node('orin_nav_odom')
    odom_ex = SingleThreadedExecutor()
    odom_ex.add_node(odom_node)

    def odom(m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        # monotonic: the Orin finds this clock's offset from its own by the packets' arrivals
        here['base'], here['base_at'] = (p.x, p.y, yaw_of((o.x, o.y, o.z, o.w))), time.monotonic()

    odom_node.create_subscription(Odometry, ODOM_TOPIC, odom, 1)

    def voxels(m):
        if 'base' not in here or time.monotonic() - here['base_at'] > 0.2 or m.point_step % 4:
            return
        # STVL publishes its grid with the camera dead too: the Orin must see the camera stop
        if time.monotonic() - here.get('camera_at', -1e9) > CAMERA_STALE_S:
            return
        xyz = np.frombuffer(bytes(m.data), np.float32).reshape(-1, m.point_step // 4)[:, :3]
        to_orin(depth_packet(here['base_at'], here['base'], xyz))

    node.create_subscription(PointCloud2, '/local_costmap/voxel_grid', voxels, 1)
    # one small message per depth frame (30 Hz), not decoded: only that it came
    node.create_subscription(CameraInfo, '/camera/depth/camera_info',
                             lambda _: here.__setitem__('camera_at', time.monotonic()), qos_profile_sensor_data,
                             raw=True)
    for i, topic in enumerate(SONARS):
        node.create_subscription(Range, topic, lambda r, i=i: to_orin(
            SONAR.pack(b'S', i, r.range, r.min_range, r.max_range, r.field_of_view)), 2)

    # --- goals -----------------------------------------------------------------------------
    def execute(gh):
        p = gh.request.pose
        with lock:
            d['seq'] = seq = (d['seq'] + 1) & 0x7fffffff
            d['active'], d['cancelling'], d['driving'] = seq, False, False     # replaces one running
        if p.header.frame_id != 'map':
            say('goal in %r, not map: refused' % p.header.frame_id)
            return finish(gh, seq, 'abort')
        q = p.pose.orientation
        x, y, yaw = p.pose.position.x, p.pose.position.y, yaw_of((q.x, q.y, q.z, q.w))
        say('goal %d: (%.2f, %.2f, %.0f deg)' % (seq, x, y, math.degrees(yaw)))
        sent, start = 0.0, time.monotonic()
        cancel_at = None
        while True:
            now = time.monotonic()
            with lock:
                if d['active'] != seq:
                    say('goal %d replaced' % seq)
                    return finish(gh, seq, 'abort')
                if gh.is_cancel_requested and cancel_at is None:
                    cancel_at = now
                    d['cancelling'], d['driving'] = True, False
                status, at = d['status'].get(seq, (None, None))
                if status == EXECUTING and cancel_at is None:
                    d['driving'] = True
            end = goal_end(status, cancel_at is not None) if status is not None else None
            if end:
                say('goal %d: the Orin says %d' % (seq, status))
                return finish(gh, seq, end)
            if cancel_at is not None and now - cancel_at > CANCEL_WAIT_S:
                say('goal %d: cancelled; the Orin did not confirm' % seq)
                return finish(gh, seq, 'canceled')
            if not status and now - start > TAKE_S:      # 0: bt_navigator has not taken it either
                say('goal %d: the Orin did not take it' % seq)
                return finish(gh, seq, 'abort')
            if status is not None and now - at > ORIN_STALE_S:
                say('goal %d: the Orin went quiet' % seq)
                return finish(gh, seq, 'abort')
            if now - sent > RESEND_S:      # UDP: until heard
                sent = now
                to_orin(CANCEL.pack(b'C', seq) if cancel_at is not None else GOAL.pack(b'G', seq, x, y, yaw))
            time.sleep(0.05)

    def finish(gh, seq, how):
        with lock:
            if d['active'] == seq:
                if d['driving'] or d['cancelling']:
                    to_orin(CANCEL.pack(b'C', seq))     # the Orin may still be on it (replaced, quiet, refused)
                d['active'], d['driving'] = None, False
        getattr(gh, how)()
        return NavigateToPose.Result()

    group = ReentrantCallbackGroup()
    ActionServer(node, NavigateToPose, 'navigate_to_pose', execute, callback_group=group,
                 cancel_callback=lambda _: CancelResponse.ACCEPT)

    # --- what comes back -----------------------------------------------------------------------
    plan_pub = node.create_publisher(Path, '/plan', 1)
    # reliable, as Nav2 publishes it: a best-effort publisher never reaches a reliable subscriber, so
    # bags (bin/run.py) recorded no /cmd_vel at all on mppi_drive2
    cmd_pub = None if dry else node.create_publisher(Twist, '/cmd_vel', 10)
    back = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    back.bind(('', DRIVE_PORT))
    back.setblocking(False)

    def send(vx, vy, wz):
        if cmd_pub is not None:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            cmd_pub.publish(t)

    def path(data):
        xy = np.frombuffer(data, np.float32, offset=PATH.size).reshape(-1, 2)
        m = Path()
        m.header.frame_id, m.header.stamp = 'map', node.get_clock().now().to_msg()
        for x, y in xy:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        plan_pub.publish(m)

    def tick():
        take_ready(odom_ex, 2)
        while True:
            try:
                data, (host, _) = back.recvfrom(65535)
            except BlockingIOError:
                break
            if host != ORIN or not data:
                continue
            k, now = data[:1], time.monotonic()
            if k == b'V' and len(data) == CMD.size:
                d['mppi'], d['at'] = CMD.unpack(data)[1:], now
            elif k == b'A' and len(data) == STATUS.size:
                _, seq, status = STATUS.unpack(data)
                with lock:
                    d['status'][seq] = (status, now)
                    for old in [s for s in d['status'] if s < seq - 10]:
                        del d['status'][old]
            elif k == b'P' and (len(data) - PATH.size) % 8 == 0 and d['driving']:
                path(data)
        now = time.monotonic()
        if d['driving']:
            send(*drive_cmd(d['mppi'], now - d['at']))
            d['zeros'] = STOP_TICKS
        elif d['zeros'] > 0:
            send(0.0, 0.0, 0.0)
            d['zeros'] -= 1

    node.create_timer(1.0 / DRIVE_HZ, tick)
    signal.signal(signal.SIGTERM, _interrupt)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    say('up%s' % (', dry: no /cmd_vel' if dry else ''))
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        seq = d['active']
        d['active'], d['driving'] = None, False
        for _ in range(STOP_TICKS):
            send(0.0, 0.0, 0.0)
            if seq is not None:
                to_orin(CANCEL.pack(b'C', seq))
            time.sleep(0.02)


def check():
    assert goal_end(SUCCEEDED, False) == 'succeed'
    assert goal_end(ABORTED, False) == 'abort'
    assert goal_end(CANCELED, False) == 'abort'         # the Orin let go on its own (perception quiet)
    assert goal_end(CANCELED, True) == 'canceled'
    assert goal_end(SUCCEEDED, True) == 'canceled'      # arrived as the cancel went: he stopped either way
    assert goal_end(EXECUTING, False) is None and goal_end(0, True) is None
    assert drive_cmd((0.7, 0.1, -2.0), 0.1) == (0.5, 0.0, -0.8)         # clamped, no strafe
    assert drive_cmd((-0.5, 0.0, 0.2), 0.1) == (-0.2, 0.0, 0.2)
    assert drive_cmd((0.2, 0.0, 0.1), 0.1) == (0.2, 0.0, 0.1)
    assert drive_cmd((0.2, 0.0, 0.1), 0.5) == (0.0, 0.0, 0.0)          # MPPI quiet
    assert drive_cmd(None, 0.0) == (0.0, 0.0, 0.0)
    import numpy as np
    # he is at (1, 2) facing +y: a voxel 1 m ahead, (1, 3), is (1, 0) around him; the floor and
    # anything over his head are dropped
    pk = depth_packet(5.0, (1.0, 2.0, math.pi / 2), np.array([[1.0, 3.0, 0.3], [1.0, 3.0, 0.0], [1.0, 3.0, 1.0]]))
    assert DEPTH.unpack(pk[:DEPTH.size])[:2] == (b'D', 5.0)
    xy = np.frombuffer(pk, np.float32, offset=DEPTH.size)
    assert xy.shape == (2,) and abs(xy[0] - 1.0) < 1e-6 and abs(xy[1]) < 1e-6, xy
    # the Orin's end reads the same layouts on the same ports
    sys.path.insert(0, __file__.rsplit('/robot/', 1)[0] + '/orin/bin')
    import importlib
    o = importlib.import_module('orin_nav')
    mine = (GOAL, CANCEL, STATUS, PATH, CMD, DEPTH, SONAR)
    assert [a.format for a in (o.GOAL, o.CANCEL, o.STATUS, o.PATH, o.CMD, o.DEPTH, o.SONAR)] == [a.format for a in mine]
    assert (o.PORT, o.DRIVE_PORT) == (ORIN_PORT, DRIVE_PORT)
    print('orin_nav ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main(dry=sys.argv[1:] == ['dry'])
