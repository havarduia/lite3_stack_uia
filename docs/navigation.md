# Navigation with Nav2

`robot/nav.py` and `robot/maps.py` on perception, `orin/bin/nav.sh` on the Orin.
Nav2 runs on a floor map only. Since 2026-11-24 the Orin's Nav2 is the only one
that drives: mapless Nav2, AMCL, the RPP/DWB controller on perception and the
shadow steps of the migration are gone (git history has them, and
`history/LIDAR_ORIN_LOG.md` their walks).

## What runs where

- **The Orin** (`orin/bin/nav.sh`, `orin/bin/orin_nav2.yaml`): Nav2 Humble plans and
  drives: bt_navigator (Humble's default tree), Smac 2D on the floor map, MPPI, and a
  collision monitor that has the last word on every command. Its map -> odom is its
  own, from the 3D localizer's fixes (`cloud_locate.py` sends each to both computers).
  Its local costmap holds the lidar (terrainAnalysis) and the depth camera (an STVL
  layer); before bt_navigator gets a goal, `orin/bin/orin_nav.py` asks the planner once
  and refuses a goal it finds no way to.
- **Perception** (`env/start_nav2_map.sh`): no Nav2 of its own since 2026-11-24.
  `robot/orin_nav.py` (the link to the Orin, the camera's points to it),
  `robot/sonar_range.py` (the sonars) and `robot/locate.py` (is he placed). Its checks
  read the floor's map and the Orin's local costmap, sent back:

| Check | Reads |
|---|---|
| a map goal or a place in a wall (`go_to_point`, `save_place`) | the floor's map (`maps.floor_grid`): walls grown by 0.2 m, half his width, and the grey never mapped are lethal |
| no route (`go_to_point`) | the Orin: it plans, empties its global costmap once (a door seen shut from further off than the lidar clears stays shut there) and plans again; still none, it refuses at once, without the tree's recoveries |
| a turn on the spot (`turn_blocked`), `cost_at`, `cost_ahead` | the Orin's local costmap (camera and lidar, 6 x 6 m, twice a second) where it reaches and is under 1 s old, else the floor's map |

- **STVL** (Spatio-Temporal Voxel Layer, on the Orin) stores depth-camera points in a 3D
  voxel grid that **decays**: 2.5 s since 2026-11-04 (it was 15 s, which kept
  bridge glare under him for 20 s; the camera is for the blind spot ahead, the
  sides are the lidar's), clears what the camera now sees empty, and clears his footprint.
- **Inflation** (local): 0.45 m (at 0.30 the planner failed next to box corners in 2
  of 3 runs), `cost_scaling_factor` 8.0 (at 3.0 the cost barely faded across
  the 0.45 m and aisles between tables looked blocked).

## Starting and stopping Nav2 safely

`nav_start()`:

1. Refuses unless standing **and the pose has settled** (hasn't moved 2 cm in
   1.5 s). Standing up jumps leg odometry by ~1.3 m in one step; a costmap
   built across that jump shows a clear path straight into a real wall.
2. Refuses without a floor: `nav_start(floor=...)`, normally through `load_floor()`.
3. Kills any leftover stack.
4. Launches `env/start_nav2_map.sh <floor>` via **`setsid`**, so the whole stack
   is in its own process group.
5. Waits for `robot.orin_nav` and for `robot/locate.py` to speak.

`nav_stop()` kills by **process group** (SIGINT → SIGTERM → SIGKILL). Why not
`pkill`? Nav2's child processes don't carry the launch name, so killing by name
misses them; widening the pattern hits `static_transform_publisher`, which
the camera and transfer services also use, and ROS launch then takes *those*
services down. So: find process groups containing a Nav2 marker, exclude any
group containing a system-service process (`NAV_NEVER`), kill the rest.
`NAV_NEVER` matches executables, not words, and launch arguments (`foo:=bar`)
are dropped before matching: the vendor's Nav2 launch line contained
`launch_realsense:=false`, and a bare "realsense" once made the guard exclude
the very group it was meant to kill. A
group counts as gone once only zombies are left in it: the dead leader of a
stack this process launched stays a zombie, and waiting on it cost 8 s a stop.

### The link to the Orin

`env/start_nav2_map.sh` copies the floor's map to the Orin and starts
`~/bin/nav.sh run FLOOR` over ssh; that lives as long as the ssh, so stopping Nav2 here
stops it there.

`robot/orin_nav.py` serves
`navigate_to_pose` instead and passes each goal and cancel to the Orin over UDP
(`orin/bin/orin_nav.py`), with the camera's points (below) and the sonars. Back come the
goal's status (and why, when the Orin found no route: `/orin_nav/refused`), the route
(published as `/plan`), the local costmap (as `/local_costmap/costmap`, re-anchored on
this odom; dropped if the two odoms disagree on his heading by more than 3°), whether
its Nav2 is up (`/orin_nav/ready`, which `set_pose` waits for) and the command, which
goes on `/cmd_vel`:

- clamped to -0.2..0.5 m/s and ±0.8 rad/s, no strafe;
- only while the goal runs here: a cancel, an estop or the goal's end stops it at once;
- a stop when the Orin has said nothing for 0.3 s, and on SIGTERM.

Each side lets go when the other goes quiet: the Orin cancels its goal after 1 s
without hearing perception, and perception ends its goal after 2 s without the Orin's
status (or 20 s without the Orin taking it, which covers its Nav2 still starting).
`ORIN_NAV_DRY=1` runs everything without publishing `/cmd_vel`, for a check at the bench.

terrainAnalysis gives each lidar point its height over the local ground (so a slope is not a
wall) and keeps its own memory: points fade after 2 s unless seen again, except within
`noDecayDis` of him (1.5 m, `orin/bin/nav.sh`), where they never fade. That memory cannot un-see:
on 2026-11-16 a 2 x 1.5 m blob drawn behind the start spot as he turned (most likely someone
standing there who then stepped away) stayed for as long as he was within 1.5 m of it, with the
live `/scan` showing the place empty, and boxed him in (sonarfix_nov16). 0.6 m was tried the same
day: a replay showed the blob gone, but on the robot he walked into things, and standing at the
start he had 11 obstacle points within 1.2 m where 1.5 m gave about 300 (nodecay_nov16), so it
was put back. Until this is solved properly: nobody within 1.5 m of him when he starts a walk.

The Orin's local costmap has the lidar (CMU terrainAnalysis, `clearDyObs` on) and, from
perception, the depth camera. `robot/orin_nav.py` decodes 10 of the driver's 30 clouds a second
(`/camera/depth/color/points`, already 5 cm voxels), turns them into base_link with the fixed
mount (`CAMERA_XYZ`, `CAMERA_PITCH`, the same numbers on both computers), keeps 0.25-3 m from
the lens and 0.05-1.0 m above base_link, at most 5000 points (one datagram). The Orin matches
each to its own pose of that moment, keeps only points ahead of his nose (x > 0.35 m: beside,
behind and under him is the lidar's job) and publishes `/camera_points` for the STVL layer,
with the camera's frames as static tf. STVL's voxels, those ahead of his nose, are the
collision monitor's `/depth_points`. Until 2026-11-24 the STVL layer was perception's, and
its voxels crossed at 5 Hz. The sonars (`/sonar/front`, `/sonar/rear`)
are not in the costmap: their ~30° cone draws walls too wide in doorways, so the front
one only stops or slows him in the collision monitor (glass). If the camera or a sonar
stops for 1 s, the Orin sends no commands, so perception publishes zeros.
A sonar at its maximum (0.80 m) saw nothing and is published as `inf`, which the collision
monitor skips. Until 2026-11-16 it went out as a reading: an arc 1.0 m ahead of base_link. The
monitor moves each reading by how far he has walked since its stamp, and its sonar readings are
0.3-0.6 s old (stamped with the last lidar scan, then queued behind the monitor's wait for the
transform), so at 0.5 m/s the arc slid 0.26 m back into the slow zone (ends at 0.75 m): about 400
half-speed slowdowns of 0.1-0.5 s per bridge walk (bridge2_nov13: 814 of 819 slowed commands had
the sonar at 0.80; the lidar and camera never had a point in the zone), none standing still.
Why the short memory: on the bridge (2026-11-04) afternoon sun glare on the glossy floor
appeared as an obstacle 0.5 m ahead; with 15 s memory he stepped onto it and stood stuck
~20 s, the camera unable to see under him to clear it.
The camera's points nearer than 0.25 m are dropped (`CAMERA_NEAR` in
`robot/orin_nav.py`, and STVL's `min_z`; was 0.05): on the bridge walk of 2026-11-13 one voxel about 12 cm
in front of the lens, where the D435 cannot measure, came and went at (-77, -39 to -41) and
he took 12 s over 3 m (4 "Optimizer fail"). The bridge was clear; the likely cause is a false
stereo match on the railing's identical bars. Not walked again since the change.
The Orin's global costmap has the map and the lidar only: a camera mark nobody saw on the
lidar once closed an open doorway (`history/LIDAR_ORIN_LOG.md` section 23).

## Reading the costmap and choosing goals

- `cost_at(x, y)`: cost of one cell (0 free … 99 lethal, `None` off-map), from the Orin's
  local costmap where it reaches, else the floor's map.
- `cost_ahead(out_to, step, settle)`: costs along the current heading. With
  `settle=N` it re-reads until two profiles a second apart agree. This is
  needed because the costmap starts empty and, with `track_unknown_space`
  off, **unseen cells read as free**: an immediate read says "clear forever",
  you pick the furthest cell, and `goto()` then refuses it as LETHAL once
  real observations land. The giveaway is cost 0 at distance 0, the robot's
  own cell, which is never really free on a populated map.
- `nav.free_ahead(profile)`: how far such a profile stays plannable, i.e. the
  last distance before the first cell `goto()` would refuse. Free means below
  LETHAL, not "low": 50 was tried first and wrongly refused open floor between
  two tables, whose inflation spreads 50–90 over the whole gap.
  `nav.goal_ahead(profile)` is that distance minus a margin, or `None` when
  too little is left; the demos pick goals with it.

## `goto(forward, heading_deg=None)`

1. Checks standing, battery, Nav2 running (on a floor, him placed), goal ≥ 0.2 m away (a goal at his
   own position gives the planner nothing to plan and aborts; use
   `turn_deg()` to rotate in place).
2. Computes the goal `forward` metres along the current heading.
3. **Refuses a LETHAL goal cell**: the planner cannot plan into one and would abort.
4. Sends a `NavigateToPose` action goal, stores the handle so `estop()` can
   cancel it, waits for the result.
5. If it succeeded and a heading was requested, corrects it with `turn()`.

Returns the action status: 4 = succeeded, 6 = aborted, 5 = cancelled.
`goal_send(gx, gy, gyaw)` is step 4 on its own: a goal at an odom point,
returned as soon as Nav2 accepts it, replacing any goal under way. No checks.
`goal_active()` says whether Nav2 is still working on it.

`goto_cancel()`, from another thread, ends a goal and leaves Nav2 up; `estop()`
ends it and kills Nav2. Either way `goto()` returns 5 and he ends halted.

## Map mode: a floor map, and going to a place on it

Being built (`.planning/plans/building-navigation.md`); what is here works
on the robot as of 2026-10-09, in one room: two routes of 2 m out and
back, stopping 0.11 to 0.18 m from the target.

```python
bot.floors()                    # ['lab']: floors that have a map
bot.load_floor('lab')           # Nav2 on that map; replaces whatever Nav2 was up
bot.set_pose(-0.2, 0.0, 0)      # "I am here", roughly: the Orin's localizer searches round it
bot.localized, bot.map_pose     # True, (x, y, yaw) on the map
bot.save_place('lab door')      # where he stands; or save_place(name, x, y, yaw_deg)
bot.go_to('lab door')           # returns the action status, like goto()
bot.go_to_point(1.9, -0.3)      # a point on the map
bot.places(); bot.rename_place(old, new); bot.delete_place(name)
```

The same from the command line: `python3 -m robot.stack floors`,
`load lab`, `here -0.2 0 0`, `save "lab door"`, `places`, `go "lab door"`,
`go 1.9 -0.3`, `forget "lab door"`.

- **The map** is one floor from the Orin's mapping run, copied with
  `env/get_map.sh` to `~/lite3_maps/<floor>/map.{pgm,yaml}` on the
  perception computer. Record it with the Orin panel's Live run, not
  "Record & map" (history/LIDAR_ORIN_LOG.md section 17). Its origin is where the
  lidar was when the run started; `base_link` was 0.20 m behind that.
  On the way `get_map.sh` makes the cells within 0.6 m of the walked path
  free floor (`env/walked_free.py`): the slicer leaves the run's start and
  cluttered rooms unseen, and unseen is a wall here. Walls stay walls,
  except where the path crosses one: he walked through it, so it is a
  door (open, or its leaf sliced as wall), and a 0.8 m gap is opened
  there. `big_walk_to_arcade` crossed 16 such cells and had no route past
  its first door without it.
- **A walk between levels** (a ramp, a slope): the Orin splits it into
  one floor per storey, and Nav2 cannot route between two floor maps. Make
  it one floor instead: on a copy of the run, `kiss_flatten.py` (takes the
  path's height out, the slope included) then `map_walls.py --one-floor`.
  The panel's Save already asks for one floor; `big_walk_to_arcade` (a
  1.4 m drop over 27 m at 3°) was made this way (history/LIDAR_ORIN_LOG.md
  section 34).
- **What runs**: [What runs where](#what-runs-where). Grey on the map
  (never seen) is a wall to the planner: routes stay on floor that was
  mapped as free. A floor needs `cloud.txt` (its walk on the Orin) for the
  3D localizer; `start_nav2_map.sh` refuses one without.
- **Telling him where he is:** `here x y deg` (the panel's I am here) sends
  the pose on `/initialpose`; `lio_relay.py` passes it to the Orin's
  localizer, which searches around it.
- **Routes are planned on the map and the lidar only.** The global
  costmap has no depth camera and no sonars (since 2026-10-14); the local
  costmap, which steers, keeps the camera, and the sonars stop or slow him in the collision monitor. With them in the global one, a depth
  mark nobody saw on the lidar (it lasts 15 s) closed an open doorway
  that the map already made narrow (`history/LIDAR_ORIN_LOG.md` section 23).
- **Two steps to start.** There is no map -> odom, and so no global
  costmap, until the Orin's localizer has placed him, so `load_floor()`
  returns with him not localized, and `set_pose()` returns when he is and
  the Orin's Nav2 is up (`/orin_nav/ready`; on a big floor its global costmap takes >30 s).
  A pose not yet taken is said again every 8 s (`POSE_AGAIN_S`), not sooner: each
  one makes the Orin's localizer search afresh, a search on a big floor takes
  2.3-2.8 s, and its own fix is too old to be used by the time it arrives, so the
  first usable fix comes about a second after it. Said again every 3 s, as before,
  he never got placed on `big_walk_to_arcade` (2026-11-13); said once, in about 7 s.
- **Localized or not** is one node's answer, `robot/locate.py`, on
  `/localized` twice a second, judged from the Orin's 3D fixes on
  `/cloud_fix`: a recent accepted fix, its walls fitting, its spread narrow
  (in a plain corridor, until he has gone 15 m since a fix that pinned every
  direction). `go_to()` refuses to start without it and stops, raising, if
  it is lost on the way.
- **No turn to face the route first** (removed 2026-11-24). It was there for the RPP
  controller, which with the goal behind him stood for 20 s and then span as a
  "recovery" (2026-10-09); MPPI's PathAngleCritic turns him along the route as he sets off.
- **Frames.** Nav2 plans on the map, so `goal_send()` and `cost_at()` move
  an odom point onto the map themselves: `goto(forward)` is in odom and
  works unchanged. `robot/orin_nav.py` refuses a goal in any other frame.
- **Places** are `~/lite3_maps/<floor>/places.json` (`robot/places.py`):
  names unique per floor whatever the case, a damaged file moved aside
  and never written over.
- `demos/check_map.py <floor> <x> <y> <deg>` draws one live scan on the
  map and finds the best-fitting pose nearby; it is how to find the pose
  to give, and the first thing to run when he will not localize.

## What he says on a map goal

A goal pressed on the panel's map (`go_point`) is spoken aloud (`robot/voice.py`'s
`say`), from fixed
lines in `robot/hmi.py`'s `SAY`: "On my way." once the goal is sent,
"I'm here." on arrival, "I couldn't get there." when it fails or is
refused, nothing when you cancel it.

Anything in the way, a shut door included, is left to Nav2: the collision
monitor stops him in front of it and the planner routes round it, or the goal fails. Until
2026-10-16 he also stopped at doors the costmap newly marked shut and asked
for them to be opened, and asked people to let him pass (`robot/doorway.py`,
removed: it cancelled and resent Nav2's goal mid-route, which seemed to confuse Nav2,
and took people and stale marks for doors; it is in git history).
