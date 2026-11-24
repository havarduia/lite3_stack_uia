#!/usr/bin/env python3
"""orin_nav - the Orin's Nav2 (Humble) and perception, joined over UDP.

Perception's robot/orin_nav.py serves the navigate_to_pose action there and passes each goal here;
this hands it to bt_navigator and sends back how it went, the route (/plan) and the command
(/cmd_vel: MPPI's, through the collision monitor). Perception publishes the command while its goal runs.

Also, for Nav2 here:
  odom -> base_link and /odom from FAST-LIO2 (/state_estimation, levelled by terrain_level);
  map -> odom from cloud_locate.py's fixes (sent to FIX_LOCAL_PORT too), each matched to the own pose
    at its FAST-LIO2 stamp: both come from the same /Odometry;
  perception's depth camera as /camera_points (the local costmap's STVL layer, which remembers and
    clears it) and /depth_points (STVL's voxels ahead of him, for the collision monitor), its sonars
    as /sonar/front and /sonar/rear;
  each goal planned once before bt_navigator gets it: with no route (after the global costmap is
    cleared once) it is refused with a reason, not left to the tree's recoveries (spins, backing up);
  the local costmap back to perception (2 Hz), for its turn checks, with whether Nav2 is up.
No command goes back while the camera, the sonars or /terrain_h are stale, and a goal is cancelled when
perception goes quiet (PERCEPTION_STALE_S): its sonars alone come 20 times a second.

    python3 orin_nav.py          system python, ROS sourced; started by nav.sh
    python3 orin_nav.py check    self-check, no ROS
"""
import collections
import math
import socket
import struct
import sys
import threading
import time

import numpy as np

from lio_relay import (CLOCK_DRIFT, FIX, FIX_LOCAL_PORT, FIX_NO_MAP, FIX_OK, MOUNT_XYZ, Q_MOUNT, inv, join, qconj,
                       qmul, rot, twist, unpack_fix, yaw_of)

PORT = 8044                 # perception's robot/orin_nav.py sends here
JETSON = '192.168.1.103'    # perception
CMD = struct.Struct('<c3d')         # V, MPPI's vx, vy, wz
DRIVE_PORT = 8045           # perception's robot/orin_nav.py listens here
POSES_KEPT = 200            # 20 s of own poses, to match a stamp from perception
MATCH_S = 0.3               # own pose at most this far from that moment, or the datagram is dropped
DEPTH = struct.Struct('<cd')        # D, perception's clock; float32 (x, y, z) around base_link follow
SONAR = struct.Struct('<cB4f')      # S, 0 front 1 rear, range, min, max, field of view
SONARS = (('front', 0.23, 0.0), ('rear', -0.31, math.pi))     # robot/sonar_range.py: x on base_link, yaw
# base_link -> camera_link, the vendor's mount (dr_nav2_mapless.launch.py: the pitch matters). The
# depth optical frame sits on camera_link (the left imager). Perception's robot/orin_nav.py turns its
# points into base_link with the same numbers (its check() compares).
CAMERA_XYZ = (0.25489, 0.0, 0.07249)
CAMERA_PITCH = 0.34907      # rad, nose down
SENSORS_STALE_S = 1.0       # camera 5 Hz, sonars 20 Hz

GOAL = struct.Struct('<cI3d')       # G, goal number, x y yaw on the map
CANCEL = struct.Struct('<cI')       # C, goal number
STATUS = struct.Struct('<cIBB')     # A, goal number, action_msgs GoalStatus (0: not taken yet), why refused
NO_ROUTE = 1                        # why: the planner found no way there, even with the costmap cleared
SUCCEEDED, ABORTED, CANCELED = 4, 6, 5
CLEAR_SETTLE_S = 2.5                # the global costmap is redrawn once a second; the map and a fresh scan are back by then
COSTMAP = struct.Struct('<c3dB3dHH')    # M, base_link x y yaw in odom, Nav2 up, origin x y, resolution, width, height; int8 cells
PATH = struct.Struct('<c')          # P; float32 (x, y) on the map follow
STEP_M = 0.1                        # route points kept this far apart
MAX_POINTS = 1000
PERCEPTION_STALE_S = 1.0
STATUS_S = 0.5                      # how often the running goal's status is told
MAP_TF_S = 0.1
MAP_TF_AHEAD_S = 0.3                # map -> odom stamped this far past the newest odom, as Nav2 expects map -> odom to postdate
FIX_MATCH_S = 0.05                  # the own pose this near the fix's stamp (FAST-LIO2 is 10 Hz)
# m: only camera voxels ahead of his nose (0.3 m, plus half of STVL's 0.1 m voxel) count. The camera is
# for the lidar's blind spot in front of him; beside and under him is the lidar's job, and the camera can't
# see under him anyway: glare on the bridge floor (bridge_nov_4_2) showed up 0.5 m ahead, he stepped onto
# it and stood stuck ~20 s while it stayed in the camera's memory.
NOSE_X = 0.35
RESYNC_S = 5.0                      # a camera stamp this much older than the offset says: perception's clock restarted


def base_of(p, q):
    """The levelled FAST-LIO2 body pose -> base_link as (x, y, yaw); the mount is base_link -> body."""
    q_base = qmul(q, qconj(Q_MOUNT))
    lever = rot(q_base, MOUNT_XYZ)
    return p[0] - lever[0], p[1] - lever[1], yaw_of(q_base)

def clock_ahead(ahead, arrived, sent):
    """This computer's clock minus perception's, from a datagram's arrival here and its stamp there: the
    quickest so far (lio_relay's way), so neither clock needs to be right or in step with the other."""
    if ahead is None or arrived - sent > ahead + RESYNC_S:
        return arrived - sent
    return min(ahead + CLOCK_DRIFT, arrived - sent)


def map_odom(fix, poses):
    """map -> odom from a fix (base_link on the map at a FAST-LIO2 stamp) and the own
    (stamp, x, y, yaw) poses in odom; None if none is near enough that stamp."""
    then = min(poses, key=lambda p: abs(p[0] - fix['stamp']), default=None)
    if then is None or abs(then[0] - fix['stamp']) > FIX_MATCH_S:
        return None
    x, y, _, yaw = fix['pose']
    return join((x, y, yaw), inv(then[1:]))


def ahead(rel):
    """Camera points around base_link (n x 2 or n x 3), only those ahead of his nose."""
    rel = np.asarray(rel, np.float32)
    return rel[rel[:, 0] > NOSE_X]


def around(base, xyz):
    """Points in odom (n x 3) -> (x, y) around base_link (x, y, yaw)."""
    c, s = math.cos(base[2]), math.sin(base[2])
    dx, dy = xyz[:, 0] - base[0], xyz[:, 1] - base[1]
    return np.stack([c * dx + s * dy, -s * dx + c * dy], 1)


def next_step(planned, cleared):
    """After the planner's answer to a new goal: 'go' (to bt_navigator), 'clear' (forget the global
    costmap's marks, ask again) or 'refuse'. A mark beyond the lidar's clearing range outlives what
    made it: a door seen shut stays shut."""
    if planned:
        return 'go'
    return 'refuse' if cleared else 'clear'


def thin(xy):
    kept = xy[:1]
    for p in xy[1:-1]:
        if math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= STEP_M - 1e-6:
            kept.append(p)
    return (kept + xy[-1:] if len(xy) > 1 else kept)[:MAX_POINTS]


def costmap_packet(base, up, m):
    """The local costmap (an OccupancyGrid in odom) and base_link of the moment -> a datagram."""
    i = m.info
    return (COSTMAP.pack(b'M', *base, up, i.origin.position.x, i.origin.position.y, i.resolution, i.width, i.height)
            + np.asarray(m.data, np.int8).tobytes())


def cloud(stamp, frame, xyz):
    from sensor_msgs.msg import PointCloud2, PointField
    c = PointCloud2()
    c.header.stamp, c.header.frame_id = stamp, frame
    c.height, c.width = 1, len(xyz)
    c.fields = [PointField(name=a, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, a in enumerate('xyz')]
    c.point_step, c.row_step, c.is_dense = 12, 12 * len(xyz), True
    c.data = np.ascontiguousarray(xyz, np.float32).tobytes()
    return c


def main():
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav2_msgs.srv import ClearEntireCostmap
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.time import Time
    from rclpy.duration import Duration
    from sensor_msgs.msg import PointCloud2, Range
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    rclpy.init()
    node = rclpy.create_node('orin_nav')
    log = node.get_logger()
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    tf_pub = TransformBroadcaster(node)
    camera_pub = node.create_publisher(PointCloud2, '/camera_points', 2)
    points_pub = node.create_publisher(PointCloud2, '/depth_points', 2)
    sonar_pubs = [node.create_publisher(Range, '/sonar/' + n, 10) for n, _, _ in SONARS]
    static_tf = StaticTransformBroadcaster(node)
    tfs = []
    for name, x, yaw in SONARS:
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'base_link', 'sonar_' + name
        t.transform.translation.x = x
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tfs.append(t)
    for parent, child, xyz, q in (
            ('base_link', 'camera_link', CAMERA_XYZ, (0.0, math.sin(CAMERA_PITCH / 2), 0.0, math.cos(CAMERA_PITCH / 2))),
            ('camera_link', 'camera_depth_optical_frame', (0.0, 0.0, 0.0), (-0.5, 0.5, -0.5, 0.5))):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = q
        tfs.append(t)
    static_tf.sendTransform(tfs)
    nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    clear = node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
    s = {'prev': None, 'stamp': None, 'map_odom': None, 'heard': -1e9, 'depth_at': -1e9, 'terrain_at': -1e9, 'ahead': None,
         'sonar_at': [-1e9, -1e9], 'blind': None,
         # the goal: its number, where, the handle once taken, its status, why refused, when to plan again
         'seq': None, 'pose': None, 'handle': None, 'status': 0, 'why': 0, 'again': None}
    by_stamp = collections.deque(maxlen=POSES_KEPT)     # (FAST-LIO2 stamp, x, y, yaw): for the fixes
    by_clock = collections.deque(maxlen=POSES_KEPT)     # (this clock, FAST-LIO2 stamp): for perception's camera
    back = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tell = lambda data: back.sendto(data, (JETSON, DRIVE_PORT))     # noqa: E731

    def odom(m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y, yaw = base_of((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        now = (t, x, y, yaw)
        vx, vy, wz = twist(s['prev'], now) if s['prev'] and 0 < t - s['prev'][0] < 1.0 else (0.0, 0.0, 0.0)
        s['prev'], s['stamp'] = now, m.header.stamp
        by_stamp.append(now)
        by_clock.append((time.monotonic(), m.header.stamp))
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = m.header.stamp, 'odom', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tf_pub.sendTransform(tf)
        o2 = Odometry()
        o2.header.stamp, o2.header.frame_id, o2.child_frame_id = m.header.stamp, 'odom', 'base_link'
        o2.pose.pose.position.x, o2.pose.pose.position.y = x, y
        o2.pose.pose.orientation = tf.transform.rotation
        o2.twist.twist.linear.x, o2.twist.twist.linear.y, o2.twist.twist.angular.z = vx, vy, wz
        odom_pub.publish(o2)

    def take_fix(f):
        if f['state'] == FIX_NO_MAP:
            if s['map_odom'] is not None:
                log.warn('cloud_locate has no map: no map -> odom')
            s['map_odom'] = None
        elif f['state'] == FIX_OK:
            m = map_odom(f, by_stamp)
            if m is not None:
                if s['map_odom'] is None:
                    log.info('map -> odom from cloud_locate')
                s['map_odom'] = m

    def map_tf():
        if s['map_odom'] is None or s['stamp'] is None:
            return
        x, y, yaw = s['map_odom']
        t = TransformStamped()
        t.header.stamp = (Time.from_msg(s['stamp']) + Duration(seconds=MAP_TF_AHEAD_S)).to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tf_pub.sendTransform(t)

    # --- goals -----------------------------------------------------------------

    def nav_up():
        return nav.server_is_ready() and planner.server_is_ready()

    def goal(seq, x, y, yaw):
        if seq == s['seq']:
            return                      # sent again until told it was taken
        if not nav_up():
            return                      # Nav2 still starting: the next one sent tries again
        p = PoseStamped()
        p.header.frame_id, p.header.stamp = 'map', node.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        s['seq'], s['pose'], s['handle'], s['status'], s['why'], s['again'] = seq, p, None, 0, 0, None
        log.info('goal %d: (%.2f, %.2f, %.0f deg)' % (seq, x, y, math.degrees(yaw)))
        ask(seq, False)

    def ask(seq, cleared):
        q = ComputePathToPose.Goal()
        q.goal, q.planner_id, q.use_start = s['pose'], 'GridBased', False
        planner.send_goal_async(q).add_done_callback(lambda fut: asked(seq, cleared, fut.result()))

    def asked(seq, cleared, handle):
        if handle is None or not handle.accepted:
            return planned(seq, cleared, False)
        handle.get_result_async().add_done_callback(
            lambda fut: planned(seq, cleared, fut.result().status == SUCCEEDED and len(fut.result().result.path.poses) > 0))

    def planned(seq, cleared, ok):
        if seq != s['seq'] or s['status'] != 0:
            return                      # replaced or cancelled while planning
        step = next_step(ok, cleared or not clear.service_is_ready())
        if step == 'go':
            g = NavigateToPose.Goal()
            g.pose = s['pose']
            nav.send_goal_async(g).add_done_callback(lambda fut: taken(seq, fut.result()))
        elif step == 'clear':
            log.info('goal %d: no route; the global costmap cleared, asked again' % seq)
            clear.call_async(ClearEntireCostmap.Request())
            s['again'] = time.monotonic() + CLEAR_SETTLE_S
        else:
            log.warn('goal %d: no route there: refused' % seq)
            s['status'], s['why'] = ABORTED, NO_ROUTE
            status_tick()

    def taken(seq, handle):
        if seq != s['seq']:
            return
        if handle is None or not handle.accepted:
            # bt_navigator turns goals down until it is active; perception gives up in its own time
            log.warn('goal %d turned down; taken again when sent again' % seq)
            s['seq'] = None             # planned again too: Nav2 may have changed since
            return
        s['handle'], s['status'] = handle, GoalStatus.STATUS_EXECUTING
        handle.get_result_async().add_done_callback(lambda fut: ended(seq, fut.result().status))

    def ended(seq, status):
        if seq == s['seq']:
            s['status'] = status
            log.info('goal %d: status %d' % (seq, status))
            status_tick()

    def cancel(seq=None):
        if seq is not None and seq != s['seq']:
            return
        if s['handle'] is not None and s['status'] == GoalStatus.STATUS_EXECUTING:
            s['handle'].cancel_goal_async()
            s['status'] = GoalStatus.STATUS_CANCELING
        elif s['seq'] is not None and s['handle'] is None and s['status'] == 0:
            s['status'], s['again'] = CANCELED, None        # still being planned: nothing to stop
            status_tick()

    def status_tick():
        if s['seq'] is not None:
            tell(STATUS.pack(b'A', s['seq'], s['status'], s['why']))

    def plan(m):
        if s['status'] not in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING) or not m.poses:
            return
        xy = thin([(p.pose.position.x, p.pose.position.y) for p in m.poses])
        tell(PATH.pack(b'P') + np.array(xy, np.float32).tobytes())

    def command(c):
        now = time.monotonic()
        blind = now - min(s['depth_at'], s['terrain_at'], *s['sonar_at']) > SENSORS_STALE_S
        if blind != s['blind']:
            s['blind'] = blind
            # two calls: rclpy raises if one call site logs at two severities, and that ended this node
            # the first time the gate tripped
            if blind:
                log.warn('camera, sonars or /terrain_h stopped coming: no commands go back')
            else:
                log.info('camera, sonars and /terrain_h coming')
        if not blind:
            tell(CMD.pack(b'V', c.linear.x, c.linear.y, c.angular.z))

    def watch():
        if time.monotonic() - s['heard'] > PERCEPTION_STALE_S and s['status'] == GoalStatus.STATUS_EXECUTING:
            log.warn('perception went quiet: goal %d cancelled' % s['seq'])
            cancel()

    # --- perception's camera and sonars ----------------------------------------

    def depth(arrived, when, rel):
        s['ahead'] = clock_ahead(s['ahead'], arrived, when)
        when += s['ahead']                      # perception's stamp, on this clock
        then = min(by_clock, key=lambda p: abs(p[0] - when), default=None)
        if then is None or abs(then[0] - when) > MATCH_S:
            return
        # stamped with the FAST-LIO2 pose of that moment: STVL places the points with odom -> base_link
        # at it, so walking does not smear them
        camera_pub.publish(cloud(then[1], 'base_link', ahead(rel)))
        s['depth_at'] = time.monotonic()

    def voxels(m):
        """STVL's memory of the camera -> /depth_points, those ahead of his nose (as perception's
        voxels were), stamped with the newest odom: the monitor looks odom -> base_link up at it (a
        later "now" would wait)."""
        if s['prev'] is None or m.point_step % 4:
            return
        xyz = np.frombuffer(m.data, np.float32).reshape(-1, m.point_step // 4)[:, :3]
        points_pub.publish(cloud(s['stamp'], 'odom', xyz[around(s['prev'][1:], xyz)[:, 0] > NOSE_X]))

    def costmap(m):
        if s['prev'] is not None:
            tell(costmap_packet(s['prev'][1:], nav_up(), m))

    def sonar(i, rng, lo, hi, fov):
        if s['stamp'] is None:
            return
        r = Range()
        # the newest odom stamp, not now(): the monitor looks the transform up at it, and a later
        # stamp made it wait for tf (mppi_drive3)
        r.header.stamp, r.header.frame_id = s['stamp'], 'sonar_' + SONARS[i][0]
        r.radiation_type, r.field_of_view = Range.ULTRASOUND, fov
        # at its maximum the sonar saw nothing: inf (REP 117), which the collision monitor skips. As a
        # reading it became an arc 1 m ahead that an old stamp pulled into the slow zone (docs/navigation.md)
        r.min_range, r.max_range, r.range = lo, hi, rng if rng < hi else math.inf
        sonar_pubs[i].publish(r)
        s['sonar_at'][i] = time.monotonic()

    inbox = collections.deque(maxlen=200)       # datagrams for the ROS thread

    def read(port, host_ok):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', port))
        while True:
            data, (host, _) = sock.recvfrom(65535)
            if host_ok(host) and data:
                inbox.append((time.monotonic(), data))

    def take():
        latest = {}         # of the camera and each sonar only the newest counts
        while inbox:
            arrived, data = inbox.popleft()
            k = data[:1]
            if k == b'L' and len(data) == FIX.size:
                take_fix(unpack_fix(data))
                continue
            s['heard'] = time.monotonic()
            if k == b'G' and len(data) == GOAL.size:
                goal(*GOAL.unpack(data)[1:])
            elif k == b'C' and len(data) == CANCEL.size:
                cancel(CANCEL.unpack(data)[1])
            elif k == b'D' and len(data) >= DEPTH.size and (len(data) - DEPTH.size) % 12 == 0:
                latest['D'] = (arrived, DEPTH.unpack(data[:DEPTH.size])[1],
                               np.frombuffer(data, np.float32, offset=DEPTH.size).reshape(-1, 3))
            elif k == b'S' and len(data) == SONAR.size:
                _, i, *r = SONAR.unpack(data)
                if i in (0, 1):
                    latest[i] = r
        if 'D' in latest:
            depth(*latest.pop('D'))
        for i, r in latest.items():
            sonar(i, *r)
        if s['again'] is not None and time.monotonic() >= s['again']:
            s['again'] = None
            ask(s['seq'], True)

    node.create_subscription(Odometry, '/state_estimation', odom, 20)
    node.create_subscription(Twist, '/cmd_vel', command, 10)
    node.create_subscription(Path, '/plan', plan, 2)
    node.create_subscription(PointCloud2, '/local_costmap/voxel_grid', voxels, 1)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', costmap, 1)
    # the lidar's obstacles (terrainAnalysis -> terrain_level), not decoded: the costmap keeps its old
    # marks when they stop, and odometry goes on
    node.create_subscription(PointCloud2, '/terrain_h', lambda _: s.__setitem__('terrain_at', time.monotonic()), 2,
                             raw=True)
    node.create_timer(0.05, take)               # the ROS calls stay on the spinning thread
    node.create_timer(MAP_TF_S, map_tf)
    node.create_timer(STATUS_S, status_tick)
    node.create_timer(0.2, watch)
    threading.Thread(target=read, args=(PORT, lambda h: h == JETSON), daemon=True).start()
    threading.Thread(target=read, args=(FIX_LOCAL_PORT, lambda h: h == '127.0.0.1'), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):      # SIGTERM; anything else shows its traceback
        pass
    finally:
        cancel()


def check():
    from lio_relay import pack_fix
    # he is at (1, 2) facing +y in odom at stamp 10.0, and the fix puts him at (5, 5) facing +x
    # on the map: map -> odom takes odom (1, 2) to (5, 5), and a point 1 m ahead, (1, 3), to (6, 5)
    poses = [(9.9, 0.0, 0.0, 0.0), (10.0, 1.0, 2.0, math.pi / 2)]
    f = unpack_fix(pack_fix(11.0, 10.0, (5.0, 5.0, 0.2, 0.0), 0.9, 0.8, 0.05, (), (0, 0, 0), FIX_OK))
    m = map_odom(f, poses)
    for odom_pt, map_pt in (((1.0, 2.0), (5.0, 5.0)), ((1.0, 3.0), (6.0, 5.0))):
        x, y, _ = join(m, odom_pt + (0.0,))
        assert abs(x - map_pt[0]) < 1e-9 and abs(y - map_pt[1]) < 1e-9, (x, y)
    f['stamp'] = 10.5
    assert map_odom(f, poses) is None            # no own pose of that moment
    assert len(thin([(i * 0.05, 0.0) for i in range(41)])) == 21
    # under him, at his nose, beside and behind him: dropped; ahead of the nose: kept
    kept = ahead([(0.0, 0.0, 0.2), (0.33, 0.1, 0.2), (0.0, 0.6, 0.2), (-1.0, 0.0, 0.2), (0.4, 0.0, 0.2), (1.0, 1.0, 0.5)])
    assert np.allclose(kept, [[0.4, 0.0, 0.2], [1.0, 1.0, 0.5]]), kept.tolist()
    assert ahead(np.zeros((0, 3), np.float32)).shape == (0, 3)
    # he is at (1, 2) facing +y: (1, 3) in odom is 1 m ahead of him
    assert np.allclose(around((1.0, 2.0, math.pi / 2), np.array([[1.0, 3.0, 0.3]])), [[1.0, 0.0]])
    # a new goal: planned -> go; no route -> clear and ask again; still none -> refused
    assert [next_step(True, False), next_step(False, False), next_step(False, True)] == ['go', 'clear', 'refuse']
    # the local costmap as perception reads it back
    from types import SimpleNamespace as NS
    pos = NS(x=-2.0, y=-1.0)
    m = NS(info=NS(origin=NS(position=pos), resolution=0.05, width=3, height=2), data=[0, 100, -1, 0, 0, 99])
    pk = costmap_packet((1.0, 2.0, 0.5), True, m)
    _, x, y, yaw, up, ox, oy, res, w, h = COSTMAP.unpack(pk[:COSTMAP.size])
    assert (x, y, yaw, up, ox, oy, w, h) == (1.0, 2.0, 0.5, 1, -2.0, -1.0, 3, 2) and abs(res - 0.05) < 1e-12
    assert np.frombuffer(pk, np.int8, offset=COSTMAP.size).tolist() == m.data
    # the two clocks 1000 s apart: the quickest packet sets the offset, a slow one does not move it,
    # and a stamp from a restarted clock (far older) starts it afresh
    a = clock_ahead(None, 1000.25, 0.0)
    a = clock_ahead(a, 1001.02, 1.0)
    assert abs(a - 1000.02) < 1e-9 and abs(clock_ahead(a, 1002.3, 2.0) - a) < 1e-3, a
    assert clock_ahead(a, 1010.0, 3.0) == 1007.0
    # standing level: the body is pitched by the mount, base_link sits MOUNT_XYZ behind/below it
    x, y, yaw = base_of((1.0 + MOUNT_XYZ[0], 2.0, 0.5 + MOUNT_XYZ[2]), Q_MOUNT)
    assert abs(x - 1.0) < 1e-9 and abs(y - 2.0) < 1e-9 and abs(yaw) < 1e-9, (x, y, yaw)
    # turned 90 deg left: the lever arm turns with him
    qz = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    x, y, yaw = base_of((1.0, 2.0 + MOUNT_XYZ[0], 0.0), qmul(qz, Q_MOUNT))
    assert abs(x - 1.0) < 1e-9 and abs(y - 2.0) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9, (x, y, yaw)
    print('orin_nav ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main()
