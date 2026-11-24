#!/usr/bin/env python3
"""nav2_map_params - the parameters for perception's half of Nav2 on a floor map.

    python3 env/nav2_map_params.py <mapless.yaml> <map.yaml> <out.yaml>
    python3 env/nav2_map_params.py check

Made from the vendor's mapless file as it stands on the robot, so its tuning
(env/nav2-mapless-lite3.patch) is done once: the global costmap moves to the
`map` frame, gets the map as its first layer and loses the depth camera and the
sonars, and a map server is added. The Orin's Nav2, which drives, has its own
file (orin/bin/orin_nav2.yaml).
"""
import copy
import sys

import yaml

# nav2_bringup's navigation launch rewrites map_subscribe_transient_local in
# whatever file it is given, to false unless told otherwise: it is set where
# that launch is included (env/nav2_map.launch.py), not here.
STATIC_LAYER = {
    'plugin': 'nav2_costmap_2d::StaticLayer',
    'map_subscribe_transient_local': True,
}
# Grey on the map is "never seen", not floor, and routes must stay on what
# was mapped as free. The static layer takes cells of this value as unknown
# and (not tracking unknown) as free; -1, the map's real unknown, is then no
# longer that value and falls through to "at least lethal_cost_threshold":
# a wall. Tracking
# unknown instead does not hold: the other layers write free over it. In
# this Nav2 the static layer reads the value from the costmap, not from itself.
UNKNOWN_COST_VALUE = 50
# The global costmap takes the map's size and cell (0.05 m), about four
# times the cells of the mapless one for a 35 m floor, so it is redone and
# sent less often. The local costmap, which steers, keeps its tuning.
# Long routes are planned on the map and the lidar only. The depth camera
# (15 s memory) and the sonars stay in the local costmap, which steers. In the
# global one they closed doorways that were open: a mark nobody saw on the
# lidar turned a route lethal (docs/history/LIDAR_ORIN_LOG.md section 23).
GLOBAL_DROPPED = ('stvl_obstacle_layer', 'range_layer')
GLOBAL_UPDATE_HZ = 1.0
GLOBAL_PUBLISH_HZ = 0.5
# The local costmap is sent 5 times a second, so a bag holds brief marks; at the
# vendor's 1 Hz it missed them.
LOCAL_PUBLISH_HZ = 5.0
# Nearer than this the D435 cannot measure: a false stereo match on the bridge railing put a voxel
# 12 cm from the lens and stopped him (docs/navigation.md, 2026-11-13). The floor enters the view at 0.5 m.
CAMERA_NEAR_M = 0.25


def map_params(mapless, map_yaml):
    """The mapless parameters (as a dict) -> map mode's."""
    p = copy.deepcopy(mapless)
    g = p['global_costmap']['global_costmap']['ros__parameters']
    g['global_frame'] = 'map'
    g['rolling_window'] = False             # the whole floor, not a window round the robot
    g['update_frequency'] = GLOBAL_UPDATE_HZ
    g['publish_frequency'] = GLOBAL_PUBLISH_HZ
    lp = p['local_costmap']['local_costmap']['ros__parameters']
    lp['publish_frequency'] = LOCAL_PUBLISH_HZ
    lp['stvl_obstacle_layer']['camera_points']['min_z'] = CAMERA_NEAR_M
    g['plugins'] = ['static_layer'] + [x for x in g['plugins'] if x not in GLOBAL_DROPPED]
    for x in GLOBAL_DROPPED:
        g.pop(x, None)
    g['static_layer'] = dict(STATIC_LAYER)
    g['unknown_cost_value'] = UNKNOWN_COST_VALUE
    p['map_server'] = {'ros__parameters': {'use_sim_time': False, 'yaml_filename': map_yaml}}
    return p


def changed(a, b, at=''):
    """The paths at which two nested dicts differ."""
    if isinstance(a, dict) and isinstance(b, dict):
        return sorted(sum((changed(a.get(k), b.get(k), at + '/' + str(k))
                           for k in set(a) | set(b)), []))
    return [] if a == b and type(a) is type(b) else [at]


def demo():
    mapless = yaml.safe_load('''
bt_navigator:
  ros__parameters: {global_frame: odom, robot_base_frame: base_link}
local_costmap:
  local_costmap:
    ros__parameters: {global_frame: odom, rolling_window: True, update_frequency: 5.0,
                      plugins: ["stvl_obstacle_layer", "range_layer", "scan_layer", "inflation_layer"],
                      stvl_obstacle_layer: {camera_points: {min_z: 0.05, max_z: 3.0}}}
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: odom
      rolling_window: True
      width: 30
      footprint: "[[0.3, 0.2], [0.3, -0.2]]"
      update_frequency: 5.0
      publish_frequency: 2.0
      track_unknown_space: False
      plugins: ["stvl_obstacle_layer", "range_layer", "scan_layer", "inflation_layer"]
      stvl_obstacle_layer: {plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer", voxel_decay: 15.0}
      range_layer: {plugin: "nav2_costmap_2d::RangeSensorLayer"}
      scan_layer: {plugin: "nav2_costmap_2d::ObstacleLayer", scan: {max_obstacle_height: 2.0}}
planner_server:
  ros__parameters: {GridBased: {allow_unknown: true, tolerance: 0.15}}
recoveries_server:
  ros__parameters: {global_frame: odom}
''')
    before = copy.deepcopy(mapless)
    out = map_params(mapless, '/maps/f/map.yaml')
    assert mapless == before                                    # the input is not touched
    g = '/global_costmap/global_costmap/ros__parameters/'
    assert changed(mapless, out) == sorted([
        '/map_server',
        g + 'global_frame', g + 'rolling_window', g + 'plugins', g + 'static_layer',
        g + 'update_frequency', g + 'publish_frequency', g + 'unknown_cost_value',
        g + 'stvl_obstacle_layer', g + 'range_layer',
        '/local_costmap/local_costmap/ros__parameters/publish_frequency',
        '/local_costmap/local_costmap/ros__parameters/stvl_obstacle_layer/camera_points/min_z']), changed(mapless, out)
    gp = out['global_costmap']['global_costmap']['ros__parameters']
    assert gp['plugins'] == ['static_layer', 'scan_layer', 'inflation_layer']   # the map first, the lidar over it
    assert 'stvl_obstacle_layer' not in gp and 'range_layer' not in gp
    lp = out['local_costmap']['local_costmap']['ros__parameters']
    assert lp['plugins'][:2] == ['stvl_obstacle_layer', 'range_layer']          # depth and sonars still steer
    assert gp['global_frame'] == 'map' and gp['rolling_window'] is False
    assert lp['global_frame'] == 'odom' and lp['publish_frequency'] == LOCAL_PUBLISH_HZ   # steering stays in odom
    assert out['recoveries_server']['ros__parameters']['global_frame'] == 'odom'
    assert out['map_server']['ros__parameters']['yaml_filename'] == '/maps/f/map.yaml'
    # written and read back it is the same, types included (a float must not come back a string)
    assert changed(out, yaml.safe_load(yaml.safe_dump(out))) == []
    print('nav2_map_params ok')


if __name__ == '__main__':
    args = sys.argv[1:]
    if args == ['check']:
        demo()
    elif len(args) == 3:
        with open(args[0]) as f:
            made = map_params(yaml.safe_load(f), args[1])
        with open(args[2], 'w') as f:
            f.write('# Made by env/nav2_map_params.py from %s. Do not edit: change that file.\n' % args[0])
            yaml.safe_dump(made, f)
    else:
        sys.exit(__doc__)
