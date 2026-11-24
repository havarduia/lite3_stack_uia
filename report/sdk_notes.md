# Notes for the report: the Python SDK for the Lite3 (`robot/lite3.py` and friends)

Gathered 2026-11-23 from the code, `docs/api.md`, `docs/hardware.md`,
`docs/background.md`, `docs/environment.md`, the demos/tests and git history.
File:line references point at the code as of commit 45be46a.

---

## 1. The problem it solves

DEEP Robotics ships the Lite3 Pro with no high-level Python library (no `robot.walk(x)`).
What you get out of the box:

- **A closed locomotion controller, `jy_exe`**, on the motion computer (RK3588, 192.168.1.120).
  It takes raw 12-byte UDP frames on port 43893 and sends telemetry to UDP 43897.
- **A vendor ROS 2 bridge, `Jetson2Motion` (`transfer_ros2.service`)**, on the perception
  computer (Xavier NX, Ubuntu 20.04, ROS 2 Foxy). It turns `/cmd_vel` into UDP and turns
  telemetry into ROS topics (`/robot_state_debug`, `/leg_odom2`, `/imu/data`, sonars, `/handle_state`).
- Very little documentation of the command codes. `robot/protocol.py` labels each code by
  where it came from:
  - **OFFICIAL**: found in DeepRoboticsLab source.
  - **COMMUNITY**: from the community repo `infodriver/lite3-sdk`, never run on this robot.
  - **Captured by us**: recorded from the handheld or the phone app.

  For the report, word the claim carefully. A community repo existed, but there was no
  vendor Python API, and the community codes were unverified.

The main problem is that the robot **ignores commands silently**. It gives no error for any of these:

| Silent failure | Cause |
|---|---|
| Velocity ignored | Not in auto mode |
| Velocity ignored | Not standing (`basic_state != 6`) |
| Velocity ignored | Left in posture ("twist body") mode: reports standing, but ignores velocity |
| Everything ignored | No controller heartbeat (interlock not held) |
| First command after boot ignored | Swallowed by `jy_exe`'s arming sequence |
| Won't stand | Battery below ~20% (stock bridge did not even publish battery) |
| ROS node discovers nothing, hangs | Default FastRTPS DDS; CycloneDDS needed |
| Robot keeps moving after Ctrl-C | Nav2's `controller_server` is a separate process publishing `/cmd_vel` |
| Stand command lies him down | Stand is a **toggle**, not a set |

The SDK's main idea (`README.md`, `docs/background.md` principle 1): **turn every silent
failure into either an automatic fix or a `Lite3Error` whose message says what to do.**

---

## 2. Shape of the SDK

```python
from robot.lite3 import Lite3, Lite3Error

with Lite3() as bot:
    bot.heartbeat_start()      # optional: Python holds the interlock instead of the handheld
    bot.stand()
    r = bot.walk(2.0, speed=0.3)
    print('moved %.2f m: %s' % (r['moved'], r['reason']))
    bot.turn_deg(90)           # positive = left
    print(bot.scan_text())     # depth camera as an ASCII bar chart
    bot.sit()
```

### Files (line counts at 45be46a)

| File | Lines | Role |
|---|---|---|
| `robot/lite3.py` | 890 | Core class: ROS node, state, heartbeat, posture, tricks, motion loop, e-stop, CLI |
| `robot/protocol.py` | 175 | Every UDP code and address, `send()`, `Lite3Error`. **No ROS**; also a raw-code CLI |
| `robot/depth.py` | 130 | Mixin: point cloud → obstacle ranges (`scan`, `clearance`, `side_clear`) |
| `robot/nav.py` | 681 | Mixin: Nav2 start/stop, costmap queries, `goto`, floor maps, places, turn sweep check |
| `demos/check_motion.py` | 428 | Regression test of the motion code against a fake robot (no hardware) |
| `demos/demo.py` | 310 | The API taught in 10 lessons |

### Class composition

`class Lite3(Nav, Depth)` (`lite3.py:207`) is built from **mixins** that share one ROS
node and one set of helpers:

- **Core** (`lite3.py`): state, posture, motion, heartbeat, e-stop.
- **`Nav`** (`nav.py`): `nav_start/nav_stop`, `cost_ahead`, `goto`, `load_floor`,
  `set_pose`, `go_to(name)`, places.
- **`Depth`** (`depth.py`): `scan`, `clearance`, `side_clear`, `scan_text`.

`robot/__init__.py` deliberately **re-exports nothing**. Importing `protocol` or `talk`
must not pull in `rclpy` (design principle 6: keep ROS optional where possible).

### Two command paths into the robot (`docs/hardware.md`)

**Path A: raw UDP frames** (`protocol.send`): `struct.pack('<IiI', code, value, 0)` sent to
`192.168.1.120:43893`. Used for posture, mode, heartbeat, posture sticks and tricks.

| Code | Meaning | Source |
|---|---|---|
| `0x21010202` | stand ↔ lie (**toggle**) | |
| `0x21010C03` / `0x21010C02` | auto / manual mode | |
| `0x21040001` | heartbeat (keepalive, ≥ 2 Hz) | |
| `0x21010D05` / `0x21010D06` | enter / leave posture mode | |
| `0x21010130` | pitch stick, −32767..32767; negative = nose up, full scale ≈14° | captured |
| `0x21010506`, `0x2101030C`, … | tricks: hello, dance, twist, twist_jump, long_jump, turn_over | dance verified; backflip left out on purpose |

Known but unused codes are kept in comments, for example the low-level joint control codes
`0x0111/0x0113/0x0114`. These bypass the stock controller, which loses control after 5 ms
without a frame.

**Path B: ROS 2 velocity.** `Twist` on `/cmd_vel` → Jetson2Motion → UDP → `jy_exe`. Nav2
and the vendor obstacle avoider (`voa`) publish to the same chain.

---

## 3. Lifecycle (`Lite3.__init__` / `close`)

When you create a `Lite3()`:

1. **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set before `rclpy` is imported**
   (`lite3.py:24`). With the default FastRTPS the node discovers nothing and hangs with no error.
2. **`rclpy.init()` and an internal `_Node`** that subscribes to everything the SDK needs:
   `/robot_state_debug`, `odom_fused`, the RealSense point cloud (on a separate node),
   global and local costmaps, both sonars, `/imu/data`, `/localized` and `/handle_state`.
3. **A background spin thread.** Callbacks only store the newest message, so `bot.battery`,
   `bot.pose` and `bot.standing` are always-current plain attributes. User code never
   writes a callback or a spin loop (design principle 3).
4. **A SIGINT (Ctrl-C) handler that runs `estop()`.**
5. **It waits up to 10 s for `/robot_state_debug`.** If nothing arrives it raises an error
   that names the likely cause: `transfer_ros2` is down, or `Lite_motion` holds UDP 43897.
6. **It sends auto mode** (skipped with `auto_mode=False`, for observing while a human drives).

**`close()`** runs as the context manager exits (`lite3.py:289`). The order matters, and
each step fixes a crash that actually happened:

1. `halt()`.
2. Stop the heartbeat. Halting comes first because nobody has seen what a standing robot
   does when it disarms.
3. Stop the spin thread first (otherwise segfault).
4. Destroy the action clients while the node still exists (otherwise `InvalidHandle`).
5. Shut down the executors, destroy the nodes, `rclpy.shutdown()`.

### Performance work in the spin loop

The bridge published odometry, IMU, state, sonars and handheld at **160 Hz each**.
Measured CPU (`docs/environment.md`):

- Taking every message cost a bare `Lite3` 35–46% of a core.
- With queue depth 1 and reading at a fixed rate: 35% at 50 Hz, **25% at 25 Hz** (chosen,
  `SPIN_HZ = 25`), 20% at 15 Hz.

Three related changes:

- **`take_ready()`** (`lite3.py:95`) runs only the callbacks that are ready, up to 12 per
  tick. It replaces `spin_once(timeout_sec=0)` in a loop, because an empty `spin_once`
  costs as much as a full one.
- **The point cloud has its own node and executor, read at 10 Hz.** At 30 Hz, unpacking it
  took half a core.
- **The global costmap is stored as raw bytes and decoded only when asked for.** Decoding
  the arcade floor's costmap took 0.43 s on the Xavier.

---

## 4. State

| Attribute | From | Meaning |
|---|---|---|
| `state` | `/robot_state_debug` | dict: basic, gait, policy, motion, task, need_move, zero_flag, battery, error, charging |
| `basic` | same | the state number that gates everything |
| `battery` | same | percent (added to the bridge by our patch) |
| `standing` | same | `basic == 6` |
| `pose` | `/odom_fused` | `(x, y, yaw)` in odom (leg odometry corrected by FAST-LIO2) |
| `attitude` | `/imu/data` | `(roll, pitch)` in degrees, quaternion → Euler |
| `ultrasound` | sonar topics | `(front, rear)` in metres |
| `nav_running` | `pgrep` | is Nav2 up |
| `status()` | all of the above | one-line summary |

### `basic_state` values seen on this robot (reverse-engineered)

| basic | Meaning | Commandable |
|---|---|---|
| 98 | just powered on: settled, **not armed** | first command is eaten |
| 8 | interlock dropped (no heartbeat): not armed | first command is eaten |
| 9 | arming, transition 8 → 9 → 1 | no |
| 1 | lying, armed | yes |
| 17, 4, 5, 7 | standing up / lying down in progress | no |
| 6 | standing | yes; the only state that accepts velocity |

Two constants encode this: `LYING = (1, 8, 98)` (settled lying states) and
`READY_STATES = (1, 6, 98)`.

---

## 5. Posture, arming and the heartbeat

- **`heartbeat_start()`** starts a thread that sends `0x21040001` at 2 Hz. Python then
  holds the interlock, so the robot obeys **with no handheld powered on**. This is the most
  important safety caveat in the project: the terminal (Ctrl-C) becomes the stop button.
- **`stand()`** (`lite3.py:486`) is idempotent, although the raw command is a toggle:
  1. Return at once if already standing.
  2. Refuse if the state is transitional, because toggling then can lie him down mid-move.
  3. Check the battery.
  4. Send the toggle and watch `basic_state`. A toggle that is acted on leaves the lying
     set within ~25 ms.
  5. If he is *still* lying after `ARM_GRACE = 2 s`, the toggle was swallowed by arming.
     Send it **once more**.

  This retry fixes "the first stand after boot does nothing". Measured 2026-09-21: the
  heartbeat alone does not arm him, mode commands do not either, and he stays at 98
  indefinitely until a real command arrives.
- **`sit()`**: halt, toggle, wait for a lying state. Idempotent.
- **`upright()`** is a context manager: heartbeat → `wait_ready()` → stand → *yield* →
  sit. He always sits first and is disarmed second (the disarm happens in `close()`).
- **`auto()`** sends "leave posture mode" before "auto mode". A robot left in posture mode
  reports standing but ignores all velocity (seen 2026-09-18).
- **`tilt(pitch_deg)`** is a context manager that pitches the body while standing still,
  used to aim the front camera at a face. It enters posture mode, re-sends the stick at
  10 Hz like the phone app does, and levels the body and leaves posture mode on exit.
- **`action(name)`** checks the trick's required start posture (`'stand'`, or lying *and
  armed* = basic 1), then sends the code 3 times at 1 Hz, like the vendor's own examples.

---

## 6. Motion: one loop for everything

`walk()`, `turn()`, `strafe()` and `steer()` all go through `_loop()` (`lite3.py:601`):

```
pre-checks: battery ≥ 20%  →  standing  →  auto mode  →  fresh pose
every 50 ms (20 Hz) until deadline (≤ HARD_TIMEOUT = 30 s):
    _safety():  handheld stick pushed since start?   → stop
                cancel requested (other thread)?      → stop
                odometry older than 0.5 s?            → stop
                roll or pitch > 30°?                  → stop
    publish Twist (clamped; NaN → 0)
    abort():    caller-specific (obstacle, lost camera)
    done():     target reached?
finally:
    halt() = 20 zero Twists over 0.4 s (best-effort QoS can drop one), settle 0.5 s
return {'reason': ..., 'start': pose, 'end': pose, ['moved' | 'turned']}
```

- **The stop guarantee.** `finally: halt()` means that on an exception, Ctrl-C or a normal
  exit, the last thing published is zero velocity.
- **Calls report rather than raise.** A walk that stops short because of an obstacle is
  normal operation, so it returns a `reason` (`'target reached'`, `'obstacle at 0.58 m'`,
  `'handheld took over - stopped'`) instead of raising (principle 4). Exceptions are kept
  for *refusals* before moving.
- **`clamp()`** (`nav.py:60`) turns a non-finite value into 0. Python's `max/min` would
  pass NaN through as the full limit, which would mean full speed.
- **Limits:** `MAX_SPEED = 0.6 m/s`, `MAX_YAW_RATE = 1.6 rad/s` commanded (he delivers
  0.7 of the commanded rate).

### The individual calls

**`walk(distance, speed=0.15, stop_distance=0.6)`**
- Distance comes from odometry: the straight-line distance from the start pose.
- If the depth camera already sees something inside `stop_distance`, it refuses to start.
- While walking it re-checks clearance at 4 Hz through `depth.sampled`, because a full
  pass over the cloud is too slow for 20 Hz. It stops on an obstacle or on a lost depth
  stream ("stopped rather than walking blind").
- Walking backwards is unguarded and says so.

**`turn(radians, rate=0.4)` / `turn_deg()`**
- **Heading is accumulated per sample.** Each step is wrapped to [−π, π], then summed.
  Wrapping the total instead saturates at π: a 180° request once spun ~409° until the
  timeout.
- **Coast compensation.** He keeps turning 0.17 s after the zero command, which overshot
  every turn by 3.5–4.5°. The turn now stops `rate × 0.17 s` early and reports the settled
  angle. ±30° requests now settle at 29.3–30.3°.
- **Side guard.** The depth camera checks the side he turns toward, with
  `TURN_SWEEP = 0.43 m` (the body corner sweeps ~0.36 m).
- **Sweep check** (added 2026-10-14). Before turning, the footprint (0.7 × 0.5 m) is swept
  through the turn in 5° steps over Nav2's local costmap. If it would cover an obstacle
  cell, the turn is refused. This was added after he turned 130° into a parked robot's tyres.

**`strafe(distance)`**: sideways, positive = left. No obstacle guard.

**`steer(control)`**: closed loop. `control()` is called every cycle and returns
`(vx, wz)`, or a string to stop. It has the same guards and the same halt. Keyboard teleop
and the web panel's joystick are built on it.

---

## 7. Emergency stop (`estop()`, `lite3.py:243`)

Ctrl-C alone is not a stop once Nav2 drives, because `controller_server` is another
process publishing its own `/cmd_vel`. `estop()` does, in order:

1. Ignore SIGINT, so a second Ctrl-C cannot abort the stop halfway.
2. Cancel the active Nav2 goal.
3. Send a zero-velocity burst.
4. **Kill the Nav2 process group.** This is the step that matters. The stack runs under
   `setsid` and is killed by group, with a deny-list (`NAV_NEVER`) that protects the bridge
   and the camera.
5. Send another zero burst, now that nothing else is publishing.

`estop(disarm=True)` also drops the heartbeat. It is opt-in because the effect on a
standing robot has never been observed.

`demos/estop_nav.py` sends a real SIGINT in the middle of a Nav2 goal and measures the
coast: **0.000 m**.

---

## 8. Depth camera → obstacles (`robot/depth.py`)

The RealSense D435i point cloud (already voxel-thinned to 5 cm by our driver patch) is
turned into "nearest obstacle per bearing":

1. **Optical frame → camera body frame:** `(x, y, z) = (zo, −xo, −yo)`.
2. **Undo the 20° nose-down mount** (`CAM_PITCH = 0.349 rad`) and add the mounting offset
   (0.255 m forward, 0.072 m up). Forgetting the pitch makes the floor look like a wall
   ~0.54 m ahead.
3. **Height filter:** keep points 8–60 cm above the floor. `base_link` is 0.33 m above the
   floor when standing. This drops the floor, and table tops he can walk under.
4. **Bin by bearing:** ±45° in 5° bins (18 bins), nearest range per bin, every other point
   sampled.

Derived checks:

- **`clearance(half_width=0.225)`** counts only points whose *lateral offset*
  `r·sin(bearing)` is inside the body half-width, and returns the forward distance
  `r·cos(bearing)`. A wall to one side is not "ahead".
- **`side_clear()`** gives the nearest obstacle in the left and right halves, for the turn
  guard.
- **`sampled()`** wraps any reading so a 20 Hz loop reads it at most every 0.25 s.
- **A stale cloud (> 1 s old) raises an error** instead of reading as "clear".

---

## 9. Command line

```
python3 -m robot.lite3 status | stand | sit | auto | walk 0.5 | turn 90 | scan |
                       nav-start | nav-stop | goto 1.5 | cost | estop [--disarm] |
                       action dance | floors | load <floor> | here x y deg [fit] |
                       places | save <name> | forget <name> | go <name> | go x y [deg]

python3 -m robot.protocol stand_toggle | hello | 0x21010300 | stick_pitch -16000 |
                          camera [on|off] | voa [on|off]       # raw frames, NO checks
```

The protocol CLI was the tool for capturing and testing unknown codes.

---

## 10. Testing

- **`demos/check_motion.py`** tests the motion code with **no robot and no network**:
  - A fake `World` replaces `Lite3._node`. Its pose follows the last Twist exactly, on a
    fake clock that only `sleep()` advances.
  - `protocol.send` is stubbed out, and a pretend Nav2 stands in for the real one.
  - Run it before and after any change to the motion code.

  What it asserts, among other things:
  - Every way out of a motion call ends halted and gives a reason.
  - A 270° turn is counted past 180° rather than wrapped.
  - The sweep check refuses a turn into a costmap obstacle.
  - `HARD_TIMEOUT` caps `steer`.
  - A crash inside `control()` still ends stopped.
  - Lying down → refused, nothing sent.
  - NaN → zero velocity.
  - Speed 0 → a `Lite3Error`, not a `ZeroDivisionError`.
  - The odometry relay dying mid-walk → stop.
  - The depth guard reads at ≤ 4 Hz.
  - A stale camera is not treated as clear.
  - Cancel from another thread ends any motion.
  - `upright()` arms before waiting and sits even after a crash.
  - Map-mode goals are in the map frame.
- **`demos/shake.py`** is an end-to-end regression test on the real robot (it moves him).
- **`demos/estop_nav.py`** proves e-stop under Nav2 (0.000 m coast).
- **`demos/demo.py`** teaches the API in 10 lessons: connect, state, perception, guards
  (the refusals are "the docs"), posture, motion, navigation, e-stop, behaviour, and no
  controller (heartbeat). Lessons 5+ need `--move`.

---

## 11. Measured constants behind the code (design principle 5: "measure, don't guess")

| Quantity | Value | Constant |
|---|---|---|
| Turn coast after stop | 0.17 s (3.5–4.5° overshoot) | `TURN_COAST_S` |
| Turn rate delivered | 0.7 × commanded | `MAX_YAW_RATE` |
| Toggle reaction / arming | ~25 ms / ~2 s | `ARM_GRACE` |
| Camera pitch | 20° down | `CAM_PITCH` |
| Standing body height | 0.33 m | `STAND_HEIGHT` |
| Body corner turning radius | ~0.36 m → guard 0.43 m | `TURN_SWEEP` |
| Pitch stick full scale | ~14° | `MAX_TILT_DEG` |
| Battery below which he won't stand | ~20% | `BATTERY_REFUSE` |
| Robot's own velocity failsafe | stops ~1.5 s after commands stop | background for the dead-man design |
| Leg odometry under-count | ×1.15 (1.712 m odometry = 1.960 m tape) | bridge patch |
| Stand-up odometry jump | ~1.3 m in one step | `pose_settled()` |

## 12. Vendor changes the SDK depends on (`env/transfer-jetson2motion.patch`)

- Adds **battery, error, charging** to `/robot_state_debug`. A flat battery used to be
  invisible: he just refused to stand.
- Publishes the **front** sonar; stock published only the rear.
- Scales leg odometry by **1.15**.
- Fixes a race where **every velocity reached the robot twice** (raw and
  obstacle-corrected), so `voa` could never veto anything.
- Caps republish rates (state, odometry and sonars at 50 Hz, joints at 10 Hz) instead of
  160 Hz, for CPU.

## 13. History (from git and `docs/background.md`)

| Date | Change |
|---|---|
| 2026-09-14 | Loose scripts replaced by one API, `lite3.py` |
| 2026-09-15 | Initial git import as a package |
| 2026-09-18 | `tilt()`, `steer()`, side check on turns |
| 2026-09-21 | Arming fix in `stand()` |
| 2026-09-24 | `lite3.py` split into `protocol` / `nav` / `depth` mixins; tricks |
| 2026-09-25 | Safety stops (tilt, handheld takeover), sonars |
| 2026-09-28 | Odometry ×1.15, turn coast fix |
| 2026-10-06 | Spin-rate / CPU work (`take_ready`, `SPIN_HZ`) |
| 2026-10-14 | Footprint sweep check before turns |
| 2026-11-13 | Last change to `lite3.py` (review fixes); 19 commits touch it |

## 14. What is built on it

- `bin/tour.py`: scripted route, stop, look, talk.
- `bin/teleop.py`: keyboard driving over SSH, through `steer()`.
- `robot/hmi.py`: the web control panel; joystick through `steer()`, Go through `go_to()`.
- `demos/*`, `robot/locate.py`, `robot/orin_nav.py`, `robot/shadow_link.py`,
  `robot/sonar_range.py`.

## 15. Limitations (honest list for the report)

- **Depth coverage.** The camera sees only ±45° ahead. Backing up is blind, `strafe` is
  unguarded, and things directly beside or behind him are invisible to the turn's side guard.
- **No sweep check without Nav2.** The sweep check needs a Nav2 costmap. Low obstacles
  beside him sit under the lidar's lowest ray.
- **Tricks.** Only `dance` is verified; the other trick codes are untested on this robot.
- **Untested behaviour.** What a standing robot does when the heartbeat drops is unknown,
  so `estop(disarm=True)` is opt-in.
- **Not pip-installable.** It depends on the ROS 2 Foxy workspaces (`rclpy`, nav2 and
  realsense messages) sourced by `env/lite3_env.sh`. Scripts add the repo root to
  `sys.path` instead.
- **One controller at a time.** Only one program may hold the robot: the panel must be
  stopped before running scripts.
- **Mode is not readable.** Nothing reports it, so `auto()` sends auto mode before every
  motion call rather than checking.

## 16. Doc drift found while gathering (fix or avoid quoting)

- `docs/api.md` says the spin thread does "`spin_once` every 50 ms". The code now uses
  `take_ready()` at 25 Hz, with the cloud at 10 Hz.
- The `Lite3.say()` docstring says "with Piper". Speech is Kokoro on the Orin now.
- `docs/hardware.md` still shows the `track` person detector, which was removed on
  2026-10-23. It also calls the perception computer "Jetson".
