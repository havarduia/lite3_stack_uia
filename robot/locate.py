"""locate - does the robot know where it is on the floor map?

    python3 -m robot.locate <floor>     the node (env/start_nav2_map.sh starts it with Nav2)
    python3 -m robot.locate check       self-check, no ROS

Publishes /localized twice a second: [state, x, y, yaw, spread_xy,
spread_yaw, fit], the pose being base_link on the map. One node works this
out so that every Lite3 (panel, command line, a script) reads the same
answer from one small topic. It is judged from the Orin's 3D fixes on
/cloud_fix (robot/cloud_locate.py; lio_relay.py makes them map -> odom): a
recent accepted fix, its walls fitting, its spread narrow (CloudJudge).

Map and its fit() are for demos/check_map.py: a 2D scan against the floor map.
"""
import collections
import math
import os
import sys
import time

import numpy as np
import yaml

from .lio_relay import CLOUD_FIX_TOPIC, FIX_NO_MAP as CLOUD_NO_MAP, FIX_OK as CLOUD_OK, \
    FIX_REFUSED as CLOUD_REFUSED, join, wrap
from lite3 import Lite3Error

TOPIC = '/localized'
SCAN_TOPIC = '/scan_walls'      # the lidar cut above the furniture: the low /scan is mostly tables and chairs
Reading = collections.namedtuple('Reading', 'state x y yaw spread_xy spread_yaw fit')
NO_POSE, LOCALIZED, SPREAD, NO_FIT, NO_SCAN, SETTLING = range(6)
WHY = {NO_POSE: 'it has not been told where it is',
       LOCALIZED: 'localized',
       SPREAD: 'its position estimate is too wide',
       NO_FIT: 'what the lidar sees does not fit the map here',
       NO_SCAN: 'no usable lidar scan',
       SETTLING: 'checking its position'}

PUBLISH_HZ = 2.0
READ_HZ = 10.0          # odometry is sampled at this, not at its 50 Hz: each message costs Python time
# demos/check_map.py's fit:
NEAR_M = 0.15           # a return this close to a mapped wall counts as on it
FIT_RANGE_M = 10.0      # further returns are left out: a small heading error moves them far
FIT_EVERY = 2           # every 2nd beam; only about half the 720 have a return at this height
FIT_MIN = 0.7           # at least this share of the returns on walls. In an empty room on paper a pose
                        # 0.2 m or 3 deg off still passes, and another room sharing a corner scores 0.62
MIN_RETURNS = 30
SPREAD_XY_M = 0.5
SPREAD_YAW = 0.35       # rad
FOUND_S = 1.0           # it must look right this long before it is called localized
# the Orin's 3D localizer (robot/cloud_locate.py, .planning/plans/cloud-localization.md)
CLOUD_OLD_S = 5.0       # no accepted 3D fix for this long: not placed (one refused fix is not lost)
CLOUD_WALLS_MIN = 0.5   # as cloudreg.ACCEPT: the walls must fit this well
WEAK_M = 15.0           # walked this far since a fix that pinned every direction (a plain corridor): too wide
LOST_S = 2.0            # and wrong this long before it is called lost: a person walking past is not that


def read_pgm(path):
    """A binary PGM as rows of bytes, top row first."""
    with open(path, 'rb') as f:
        data = f.read()
    fields, i = [], 0
    try:
        while len(fields) < 4:
            while data[i:i + 1].isspace():
                i += 1
            if data[i:i + 1] == b'#':
                i = data.index(b'\n', i)
                continue
            j = i
            while not data[j:j + 1].isspace():
                j += 1
            fields.append(data[i:j])
            i = j
        w, h, top = int(fields[1]), int(fields[2]), int(fields[3])
        if fields[0] != b'P5' or top > 255:
            raise ValueError('not an 8-bit binary PGM')
        return np.frombuffer(data, np.uint8, w * h, i + 1).reshape(h, w)
    except (ValueError, IndexError) as e:
        raise Lite3Error('cannot read the map picture %s: %s' % (path, e))


def grow(occupied, cells):
    """Each occupied cell spread `cells` cells up, down, left and right."""
    out = occupied
    for _ in range(cells):
        g = out.copy()
        g[1:] |= out[:-1]
        g[:-1] |= out[1:]
        g[:, 1:] |= out[:, :-1]
        g[:, :-1] |= out[:, 1:]
        out = g
    return out


class Map:
    """A floor map in Nav2's format, as the cells within NEAR_M of a wall."""

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            info = yaml.safe_load(f)
        self.res = float(info['resolution'])
        self.x0, self.y0, turned = (float(v) for v in info['origin'])
        if turned:
            raise Lite3Error('the map %s is turned (origin yaw %s); only 0 is read' % (yaml_path, turned))
        picture = read_pgm(os.path.join(os.path.dirname(yaml_path), info['image']))
        dark = picture / 255.0 if info.get('negate') else (255 - picture.astype(np.float32)) / 255.0
        # the picture's top row is the map's far edge: row 0 here is y0
        self.near = grow((dark > float(info.get('occupied_thresh', 0.65)))[::-1].copy(),
                         int(round(NEAR_M / self.res)))

    def fit(self, pose, angle_min, angle_increment, ranges):
        """The share of a scan's returns that land on the map's walls, the scan
        taken at `pose` (x, y, yaw of base_link on the map). None if there
        are too few returns to say."""
        r = np.asarray(ranges, dtype=np.float32)[::FIT_EVERY]
        a = pose[2] + angle_min + angle_increment * FIT_EVERY * np.arange(len(r))
        ok = np.isfinite(r) & (r > 0) & (r < FIT_RANGE_M)
        if ok.sum() < MIN_RETURNS:
            return None
        r, a = r[ok], a[ok]
        cx = np.floor((pose[0] + r * np.cos(a) - self.x0) / self.res).astype(int)
        cy = np.floor((pose[1] + r * np.sin(a) - self.y0) / self.res).astype(int)
        h, w = self.near.shape
        on_map = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)     # off the map is a miss
        return float(self.near[cy[on_map], cx[on_map]].sum()) / len(r)


class CloudJudge:
    """Localized or not, from /cloud_fix rows (lio_relay.CloudFix.take).
    take() each row with the odometry (x, y) of the moment; judge() any time."""

    def __init__(self):
        self.fix = None         # map -> odom, the newest accepted
        self.ok_at = None
        self.walls, self.spread, self.weak = math.nan, (math.nan, math.nan), False
        self.pinned_at = None   # odometry (x, y) at the last fix with no weak direction

    def take(self, now, row, odom_xy):
        state = int(row[0])
        if state == CLOUD_NO_MAP:
            self.__init__()
            return
        if not math.isnan(row[1]):
            self.fix = (row[1], row[2], row[3])
        if state == CLOUD_OK:
            self.ok_at, self.walls, self.spread, self.weak = now, row[4], (row[7], row[8]), int(row[6]) != 0
            if not self.weak or self.pinned_at is None:
                self.pinned_at = odom_xy

    def judge(self, now, odom_xy):
        if self.fix is None:
            return NO_POSE
        if self.ok_at is None or now - self.ok_at > CLOUD_OLD_S:
            return NO_SCAN
        if self.pinned_at is not None and math.hypot(odom_xy[0] - self.pinned_at[0],
                                                     odom_xy[1] - self.pinned_at[1]) > WEAK_M:
            return SPREAD
        # with a weak direction its variance is huge by design: the distance rule above covers it
        if not self.weak and (self.spread[0] > SPREAD_XY_M or self.spread[1] > SPREAD_YAW):
            return SPREAD
        return LOCALIZED if self.walls >= CLOUD_WALLS_MIN else NO_FIT


class Steady:
    """Localized or not, changing only once the new answer has held:
    FOUND_S to become yes, LOST_S to become no."""

    def __init__(self):
        self.yes = False
        self.since = None

    def update(self, now, looks):
        if looks == self.yes:
            self.since = None
        else:
            if self.since is None:
                self.since = now
            if now - self.since >= (FOUND_S if looks else LOST_S):
                self.yes, self.since = looks, None
        return self.yes


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Float64MultiArray

    from lite3.robot import take_ready
    from .stack import ODOM_TOPIC
    from .lio_relay import yaw_of

    rclpy.init()
    node = rclpy.create_node('locate')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    pub = node.create_publisher(Float64MultiArray, TOPIC, 1)
    new = {}
    node.create_subscription(Odometry, ODOM_TOPIC, lambda m: new.__setitem__('odom', m), 1)
    node.create_subscription(Float64MultiArray, CLOUD_FIX_TOPIC, lambda m: new.__setitem__('fix', list(m.data)), 5)
    judge3d, steady, odom, tick = CloudJudge(), Steady(), None, 0
    try:
        while rclpy.ok():
            take_ready(executor)
            m = new.pop('odom', None)
            if m is not None:
                q = m.pose.pose.orientation
                odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of((q.x, q.y, q.z, q.w)))
            now = time.monotonic()
            row = new.pop('fix', None)
            if row is not None and odom is not None:
                judge3d.take(now, row, odom[:2])
            tick += 1
            if tick % int(READ_HZ / PUBLISH_HZ) == 0:
                pose = (math.nan,) * 3
                looks = NO_POSE
                if odom is not None:
                    looks = judge3d.judge(now, odom[:2])
                    if judge3d.fix is not None:
                        x, y, yaw = join(judge3d.fix, odom)
                        pose = (x, y, wrap(yaw))
                state = LOCALIZED if steady.update(now, looks == LOCALIZED) else \
                    (SETTLING if looks == LOCALIZED else looks)
                pub.publish(Float64MultiArray(data=[float(state)] + list(pose) + list(judge3d.spread) + [judge3d.walls]))
            time.sleep(1.0 / READ_HZ)
    except KeyboardInterrupt:
        pass


def demo():
    import tempfile

    def room(w, h, res=0.05, margin=1.0):
        """A map of an empty room, its corner at (0, 0), in a temporary folder."""
        cols, rows = int((w + 2 * margin) / res), int((h + 2 * margin) / res)
        px = np.full((rows, cols), 254, np.uint8)
        c0, c1, r0, r1 = int(margin / res), int((margin + w) / res), int(margin / res), int((margin + h) / res)
        px[r0, c0:c1 + 1] = px[r1, c0:c1 + 1] = 0
        px[r0:r1 + 1, c0] = px[r0:r1 + 1, c1] = 0
        folder = tempfile.mkdtemp()
        with open(os.path.join(folder, 'map.pgm'), 'wb') as f:
            f.write(b'P5\n# made for a check\n%d %d\n255\n' % (cols, rows) + px[::-1].tobytes())
        with open(os.path.join(folder, 'map.yaml'), 'w') as f:
            f.write('image: map.pgm\nresolution: %s\norigin: [%s, %s, 0.0]\nnegate: 0\n'
                    'occupied_thresh: 0.65\nfree_thresh: 0.15\n' % (res, -margin, -margin))
        return os.path.join(folder, 'map.yaml')

    def scan_in(w, h, pose, n=720):
        """What a lidar at `pose` in that room would measure."""
        out = []
        for i in range(n):
            a = pose[2] - math.pi + i * 2 * math.pi / n
            c, s = math.cos(a), math.sin(a)
            tx = ((w if c > 0 else 0.0) - pose[0]) / c if abs(c) > 1e-9 else math.inf
            ty = ((h if s > 0 else 0.0) - pose[1]) / s if abs(s) > 1e-9 else math.inf
            out.append(min(tx, ty))
        return -math.pi, 2 * math.pi / n, out

    m = Map(room(12.0, 9.0))
    assert m.res == 0.05 and (m.x0, m.y0) == (-1.0, -1.0) and m.near.shape == (220, 280)
    cell = lambda x, y: m.near[int((y - m.y0) / m.res), int((x - m.x0) / m.res)]     # noqa: E731
    assert cell(0.0, 4.0) and cell(0.1, 4.0) and not cell(0.3, 4.0) and not cell(6.0, 4.5)
    assert cell(6.0, 9.0) and not cell(6.0, 8.5)        # the far wall is at the far y: rows not upside down

    here = (4.0, 3.0, 0.5)
    scan = scan_in(12.0, 9.0, here)
    assert m.fit(here, *scan) > 0.95
    assert m.fit((here[0], here[1], here[2] + math.radians(15)), *scan) < 0.3     # turned
    assert m.fit((here[0] + 0.5, here[1] + 0.5, here[2]), *scan) < 0.2            # moved off both ways
    # another floor's room: nothing fits, unless two of its walls happen to lie on two of ours
    assert m.fit((6.0, 4.5, 0.5), *scan_in(8.0, 5.0, (4.0, 2.5, 0.5))) == 0.0
    assert 0.5 < m.fit((2.0, 2.0, 0.5), *scan_in(8.0, 5.0, (2.0, 2.0, 0.5))) < FIT_MIN
    # the blind spot: moved along one pair of walls, the other pair still fits
    assert 0.3 < m.fit((here[0] + 0.5, here[1], here[2]), *scan) < 0.7
    assert m.fit((500.0, 500.0, 0.0), *scan) == 0.0                               # off the map: all misses
    nothing = [math.inf] * 720
    assert m.fit(here, scan[0], scan[1], nothing) is None                         # nothing in range: cannot say
    assert m.fit(here, scan[0], scan[1], nothing[:400] + scan[2][400:]) > 0.95    # "nothing there" beams do not count

    s = Steady()
    assert not s.update(0.0, True) and not s.update(0.9, True) and s.update(1.0, True)    # held FOUND_S
    assert s.update(5.0, False) and s.update(6.9, False)                # someone walks past: still localized
    assert s.update(7.0, True) and s.update(8.0, False) and s.update(9.9, False)   # and the count starts again
    assert not s.update(10.0, False)                                    # wrong for LOST_S: lost
    assert not s.update(10.5, True) and not s.update(10.6, False) and not s.update(11.4, True)
    # judged from the Orin's 3D fixes (/cloud_fix rows from lio_relay.CloudFix)
    def row(state, mo=(5.0, 5.0, 0.0), walls=0.95, weak=0, sxy=0.05, syaw=0.01):
        return [float(state)] + list(mo) + [walls, 0.05, float(weak), sxy, syaw, 0.9]
    c = CloudJudge()
    assert c.judge(0.0, (0.0, 0.0)) == NO_POSE
    c.take(0.0, row(CLOUD_OK), (0.0, 0.0))
    assert c.judge(0.5, (0.0, 0.0)) == LOCALIZED and c.fix == (5.0, 5.0, 0.0)
    c.take(1.0, row(CLOUD_REFUSED, walls=0.2), (0.1, 0.0))        # one refused fix: still placed
    assert c.judge(1.5, (0.1, 0.0)) == LOCALIZED
    assert c.judge(0.0 + CLOUD_OLD_S + 0.1, (0.1, 0.0)) == NO_SCAN    # no accepted fix for CLOUD_OLD_S
    c.take(7.0, row(CLOUD_OK, walls=0.3), (0.2, 0.0))
    assert c.judge(7.1, (0.2, 0.0)) == NO_FIT                       # the walls do not fit
    c.take(8.0, row(CLOUD_OK, sxy=0.8), (0.2, 0.0))
    assert c.judge(8.1, (0.2, 0.0)) == SPREAD                       # too wide where nothing is weak
    # a plain corridor: along it is weak (huge variance there), fine until he has gone WEAK_M on it
    c.take(9.0, row(CLOUD_OK), (0.0, 0.0))
    c.take(10.0, row(CLOUD_OK, weak=1, sxy=1e3), (5.0, 0.0))
    assert c.judge(10.1, (5.0, 0.0)) == LOCALIZED
    c.take(11.0, row(CLOUD_OK, weak=1, sxy=1e3), (WEAK_M + 1.0, 0.0))
    assert c.judge(11.1, (WEAK_M + 1.0, 0.0)) == SPREAD
    c.take(12.0, row(CLOUD_OK), (WEAK_M + 2.0, 0.0))                # pinned again: fine
    assert c.judge(12.1, (WEAK_M + 2.0, 0.0)) == LOCALIZED
    c.take(13.0, row(CLOUD_NO_MAP, mo=(math.nan,) * 3), (WEAK_M + 2.0, 0.0))
    assert c.fix is None and c.judge(13.1, (0.0, 0.0)) == NO_POSE   # the Orin has no map
    print('locate ok')


if __name__ == '__main__':
    if sys.argv[1:] == ['check']:
        demo()
    elif len(sys.argv) == 2:
        main()                          # the floor is the Orin's to know (cloud.txt); kept for the process name
    else:
        sys.exit(__doc__)
