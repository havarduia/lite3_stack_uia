# `StackRobot`: the SDK plus this project, and the sonars for Nav2

The robot itself (state, posture, motion, the depth camera, the speaker, the
emergency stop and the arming interlock) is the SDK, `lite3_sdk_uia`: read
its `docs/api.md` and `docs/robot.md`. This page is what the stack adds.

## Structure (`robot/stack.py`)

```python
class StackRobot(Maps, Nav, Voice, Lite3):   # Lite3 from the SDK
```

| Part | File | Adds |
|---|---|---|
| `Lite3` | SDK `lite3/robot.py` | everything the stock robot can do |
| `Nav` | `robot/nav.py` | `nav_start`, `nav_stop`, `nav_running`, `cost_ahead`, `cost_at`, `goto`, `goal_send`, `goto_cancel`, the turn sweep check |
| `Maps` | `robot/maps.py` | `floors`, `load_floor`, `localized`, `map_pose`, `set_pose`, `route_to`, `go_to_point`, `go_to`, places ([navigation.md](navigation.md)) |
| `Voice` | `robot/voice.py` | `say()`, Kokoro on the Orin ([voice.md](voice.md)) |

What `StackRobot` changes in the SDK:

- **Odometry:** `/odom_fused`, the lidar odometry from the Orin
  ([Lidar and FAST-LIO2 odometry (robot/lio_relay.py)](lidar.md)), instead of the
  SDK's leg odometry. Stale odometry stops every motion call, as in the SDK.
- **Its node** (`StackNode`) also listens to Nav2's global and local costmaps
  and to `robot/locate.py` (`/localized`, map mode).
- **`turn_blocked()`** is the sweep check below, instead of the SDK's "never".
- **`estop()`** cancels the Nav2 goal, halts, kills the Nav2 stack, then runs
  the SDK's estop (cancel, halt, and `disarm=True` drops the heartbeat).
  Ctrl-C alone is not a stop once Nav2 drives: its `controller_server` is a
  separate process publishing its own `/cmd_vel`, so zero commands from this
  process only race it. `demos/estop_nav.py` fires a real SIGINT mid-goal and
  measures the coast; it measured **0.000 m**.
- **`status()`** adds `nav2=up|down` and, on a floor map, where he is on it.
- **`close()`** destroys the Nav2 action clients through the SDK's
  `destroy_clients()` hook, after the spin thread and before the node.

## The turn sweep check

Before turning (2026-10-14), his body (0.7 x 0.5 m, the
  Nav2 footprint with its padding) is swept through the turn in 5° steps
  over Nav2's **local** costmap (depth, sonars and lidar; the global one if
  there is no local). If it would cover an obstacle cell it does not cover
  already, the turn is refused: `turned` 0 and a reason saying how far in
  it would touch. Added after he turned 130° on the spot, at the end of a
  `go start`, into a parked robot's tyres beside him
  (`history/LIDAR_ORIN_LOG.md` section 24). It does not offer the other way round:
  with a rectangular body, anything near enough to touch one way is swept
  the long way too. It sees only what the costmap has: low things beside
  him are under the lidar's lowest side ray (plan task 11a). With Nav2
  down there is no costmap and no sweep check.

## Command line

```
python3 -m robot.stack status | stand | sit | walk 0.5 | turn 90 | scan |
                       nav-start | nav-stop | goto 1.5 | cost | estop | action dance
on a floor map: floors, load <floor>, here <x> <y> <deg> [fit], places,
                save <name> [x y deg], forget <name>, go <name>, go <x> <y> [deg]
```

## Sonars → Nav2 (`robot/sonar_range.py`)

The robot has front and rear ultrasonic sensors. `Jetson2Motion` publishes
them as bare numbers (`Float64`) at ~160 Hz; Nav2's range layer needs
`sensor_msgs/Range` in a TF frame. This small node:

- republishes both at 20 Hz as `/sonar/front` and `/sonar/rear`;
- publishes static transforms `base_link → sonar_front` (x +0.23 m) and
  `sonar_rear` (x −0.31 m, rotated 180°);
- stamps each `Range` with the latest odometry timestamp, because the robot's
  TF uses the steady clock and a mismatched stamp makes the transform fail;
- clamps readings: the front is cut to **0.8 m max**, the rear keeps 4.0 m.
  A reading at or past the maximum is published *as* the maximum, which the
  range layer reads as "nothing there".

Measured 2026-09-27/28 with tape and the depth camera: readings are good to
~2 cm; 0.28 m is the minimum ("something within ~0.3 m") and, seen lying
down, also "no echo", which is why the SDK exposes the rear reading without
using it as a guard; 4.5 (sometimes ~4.7) means no echo; a tilted surface can
deflect the pulse and read far. Objects ~1 m away at ±34° are caught
intermittently and at 27–32° steadily, hence the ±33° beam. The front sonar's
position comes from the camera-sonar offset; the rear is not measured and is
placed at the back of the body (nose 0.274 m ahead of `base_link`, body
0.61 m long).

Why cut the front? The beam was measured at ~±33° wide. Nav2's range layer
spreads a reading over the whole cone, so a chair 30° off to the side at 1 m
became a phantom wall across the aisle. The depth camera handles that range
well; the front sonar is kept for what the camera misses up close (< ~0.6 m,
glass). The rear sonar is the *only* sensor behind the robot, so it keeps its
full range.

It writes `/tmp/sonar_range.ready` on first publish, and
`start_nav2_mapless.sh` waits for that file before launching Nav2, otherwise
Nav2 complains "No range readings received" on every start.
