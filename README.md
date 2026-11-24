# lite3-robot

Navigation, maps, speech, a web panel and tools for a **DEEP Robotics Lite3
Pro** quadruped ("robot dog"). It runs on the robot's perception computer
(Xavier NX, Ubuntu 20.04, ROS2 Foxy, Python 3.8) and on an Orin Nano we added
(`orin/`), on top of the robot SDK
[lite3_sdk_uia](https://github.com/havarduia/lite3_sdk_uia) (`import lite3`), which does everything the
stock robot can: posture, motion, state, depth camera, sonars, speaker.

```python
from robot.stack import StackRobot

with StackRobot() as bot:
    bot.stand()
    bot.walk(1.0)              # stops by itself if the depth camera sees an obstacle
    bot.turn_deg(90)           # positive = left
    print(bot.scan_text())     # what the depth camera sees, as a bar chart
```

`StackRobot` is the SDK's `Lite3` plus what this project added: lidar
odometry from the Orin, Nav2 navigation on a floor map (the Orin plans and
drives with MPPI), named places, and his voice. On top of it: keyboard teleop
and a web control panel.

## The idea

Three computers ([docs/hardware.md](docs/hardware.md)):

```
 Orin Nano (added, Humble)   lidar, FAST-LIO2, 3D localizer, Nav2 (Smac + MPPI + collision
        ▲   │                monitor, camera in its costmap), his voice (Kokoro)
        │   │ UDP: goals, camera points, sonars ▲ / status, route, costmap, cmd_vel ▼
 perception (vendor's, Foxy) lite3_sdk + StackRobot, relays, web panel, RealSense driver
        │
 motion (vendor's)           jy_exe: balance and gait, front camera, speaker
```

All the navigation is **bolted on**: it lives on the Orin, and perception only relays to
it. Take the Orin away and the robot is close to stock again: the legs alone give the
odometry, and the SDK and the panel still work.

The **motion computer** runs the vendor's closed controller (`jy_exe`) that balances
and walks. The **perception computer** runs ROS2, the depth camera, the SDK and this
repository's relays and panel. The **Orin** does everything that needs the lidar.

Out of the box the robot ignores commands without saying why: wrong mode, lying down, low
battery, not "armed". The SDK's `Lite3` class turns each of those silent failures
into an automatic fix or a clear `Lite3Error`. It keeps the robot's state
as plain attributes (`bot.battery`, `bot.pose`, `bot.standing`). Every
motion call blocks, checks safety every 50 ms and always ends with the robot
stopped, even after a crash or Ctrl-C.

## Safety first

- **The handheld remote is the only hardware stop.** Keep it in your hand:
  touching a stick takes him over (manual mode) and stops any program's motion.
- `StackRobot()` holds the heartbeat itself, so the robot obeys your program
  with **no handheld at all**, and your terminal (Ctrl-C) is the stop. It always
  does: with only the app's heartbeat, an app that went off mid-walk let him walk
  on and then lie down mid-stride, nearly falling (2026-11-24).
- Ctrl-C in any `StackRobot` program runs `estop()`, which also kills Nav2.
- `demos/estop_nav.py` and `demo.py ... --move` **move the robot**.

## Getting started

1. SSH to the perception computer: `ssh ysc@lite3-perception` (Tailscale),
   or `ssh -J ysc@192.168.2.1 ysc@192.168.1.103` from the robot's own wifi.
   The repository is in `~/robot` there.
2. The SDK must be installed there once (`~/lite3_sdk`, its `docs/setup.md`):

   ```bash
   pip3 install --user --no-build-isolation -e ~/lite3_sdk
   ```

3. Set up the shell. Do this in every new shell:

   ```bash
   source ~/robot/env/lite3_env.sh
   ```

4. Look, without moving anything:

   ```bash
   python3 -m robot.stack status        # one line: battery, posture, pose, Nav2, sonars
   python3 -m robot.stack scan          # the depth camera as a bar chart
   python3 demos/demo.py read           # the API tour, the lessons that do not move
   ```

5. Move, with the handheld in your hand and space around the robot:

   ```bash
   python3 -m robot.stack stand
   python3 -m robot.stack walk 0.5
   python3 -m robot.stack sit
   ```

6. Open the web panel at `http://lite3-perception:8080` (Tailscale) or
   `http://192.168.2.1:8080` (the robot's wifi); it has no login, so only the
   tailnet and his wifi keep it to operators (PRIVACY.md). The panel runs as
   `hmi.service`. Only one program may hold the robot: stop the panel
   (`sudo systemctl stop hmi`) before you run other scripts.

Your first program:

```python
from lite3 import Lite3Error
from robot.stack import StackRobot

with StackRobot() as bot:
    bot.wait_ready()             # it holds the heartbeat: with no app, YOU are the stop
    bot.stand()
    try:
        r = bot.walk(2.0, speed=0.3)
        print('moved %.2f m: %s' % (r['moved'], r['reason']))
    except Lite3Error as e:
        print('refused:', e)     # the message says what to do
    finally:
        bot.sit()
```

Before and after a change to `robot/stack.py`, `nav.py` or `maps.py`, run
`python3 demos/check_motion.py` (and `demos/check_voice.py` for the voice).
They test against the SDK's fake robot, so they run anywhere. The motion loop
itself is the SDK's, with its own checks in `lite3_sdk_uia/tests/`.

## Where to read next

| To... | Read |
|---|---|
| understand the computers, the network ports and the arming interlock | [docs/hardware.md](docs/hardware.md) |
| use the robot itself: state, posture, motion, e-stop, depth camera, speaker | the SDK's `docs/api.md` and `docs/robot.md` |
| see what `StackRobot` adds, and the sonars | [docs/api.md](docs/api.md) |
| navigate with Nav2 on a floor map, and save places | [docs/navigation.md](docs/navigation.md) |
| make the robot talk | [docs/voice.md](docs/voice.md) |
| run or change the web panel, cameras and wifi networking | [docs/panel.md](docs/panel.md) |
| work with the lidar, the Orin and FAST-LIO2 odometry | [docs/lidar.md](docs/lidar.md) |
| run teleop, demos, rviz, or measure a run | [docs/tools.md](docs/tools.md) |
| set up the environment, see our vendor patches and measured numbers | [docs/environment.md](docs/environment.md) |
| know why it is built this way, FAQ, glossary | [docs/background.md](docs/background.md) |

Taking over the robot? Start with [HANDOVER.md](HANDOVER.md): the computers,
logins, and what is tied to a person's accounts. The robot films people:
[PRIVACY.md](PRIVACY.md) says what it collects and how to run it in public.

Lab notes and old plans are in [docs/history/](docs/history/README.md).

## Repository layout

```
robot/                 the library: `from robot.stack import StackRobot`
  stack.py             StackRobot: the SDK's Lite3 + Nav, Maps, Voice; its CLI
  nav.py               Nav2 start/stop, costmap, goto(), the turn sweep check (part of StackRobot)
  maps.py              floor maps, where he is on one, places, go_to()         (part of StackRobot)
  voice.py             text-to-speech (Kokoro on the Orin) through the SDK's speaker (part of StackRobot)
  protocol.py          the stack's addresses; no ROS; also a raw-code CLI
  orin_nav.py          the link to the Orin's Nav2: goals, camera points, sonars out; status, cmd_vel back
  sonar_range.py       ROS node: sonars -> sensor_msgs/Range for the Orin's collision monitor
  hmi.py               web control panel server; the page is in hmi_static/
  rs_stream.py         RealSense colour -> H.264 video          (started by hmi.py)
  udp_relay.py         WebRTC video to the panel                 (started by hmi.py)
  rviz.py              the live robot in rviz2 (run it on the laptop)
  lio_relay.py         FAST-LIO2's pose from the Orin -> /odom_fused; lidar -> /scan
  locate.py            is the robot localized on the floor map? -> /localized
  places.py            named places per floor, in ~/lite3_maps/<floor>/places.json
  mapmaker.py          a floor map from one walk (the panel's Record map)
  cloud_locate.py      where he is on a floor's 3D map, from lidar scans  (runs on the Orin)
  cloudreg.py          lidar scan to 3D map matching (ICP), used by cloud_locate.py
orin/                  the Orin: deploy.sh copies orin/bin/ to its ~/bin; its own README.md
  bin/nav.sh, orin_nav2.yaml, orin_nav.py   its Nav2 and the link to perception
  bin/lio_*.py, lidar_*.py, map_*.py        FAST-LIO2 runs, the lidar panel (:8000), floor maps
  loadwatch.py         CPU and memory log of one computer   (used by bin/run.py)
  frontcam.py          front camera frames during a run      (used by bin/run.py)
bin/                   things you run
  teleop.py            drive from the keyboard over SSH
  run.py               record a test run on all three computers   (laptop)
  report.py            numbers and a plot from a recorded run      (laptop)
  tags.py              AprilTag sheets and survey, for ground truth (laptop)
  locate_replay.py     localize a recorded walk on another walk's map, offline
demos/                 teaching and regression scripts
  demo.py              the API in 10 lessons
  check_motion.py      StackRobot, Nav2 and maps against a fake robot (no robot needed)
  check_voice.py       the voice on the SDK's speaker              (no robot needed)
  check_map.py         one live lidar scan drawn on a floor map
  estop_nav.py         proves e-stop works under Nav2              (MOVES THE ROBOT)
env/                   environment, launchers, services, and patches to vendor code
  lite3_env.sh         source this first: ROS workspaces, CycloneDDS, PYTHONPATH
  *.patch              our changes to vendor code (docs/environment.md)
  start_*.sh, *.launch.py                       how Nav2 (the link to the Orin's) and the camera are started
  get_map.sh, floor_free.py, walked_free.py     fetch a floor map from the Orin and clean it
  *.service, *.conf, *.sh                       systemd units; each file's header says
                                                which computer it is for and how to install it
  tags/                surveyed AprilTag positions per floor
urdf/                  the Lite3 model and meshes for rviz and the panel
docs/                  the documentation; docs/history/ has lab notes
```

`robot/__init__.py` re-exports nothing on purpose: `robot.voice` and
`robot.protocol` must import without `rclpy`.

## Things that will bite you

- **`realsense_ros2.service` does not come back after a reboot.** Start it
  with `python3 -m robot.protocol camera on`, or anything that uses depth
  raises "no point cloud".
- **Only one program may hold the robot at a time.** The panel holds a
  `Lite3` with the heartbeat running.
- **`Lite3()` times out waiting for state?** Usually `transfer_ros2` and the
  vendor's `Lite_motion` are fighting over UDP 43897.
- **Stand before you start Nav2**, and let the pose settle. `nav_start()`
  enforces this.
- **Touching a handheld stick** puts the robot in manual mode. Motion calls
  stop with "handheld took over".
- **No odometry, no motion.** If `lio_relay.service` is down, motion calls
  refuse to start (docs/lidar.md).
- Below **~0.2 m/s** the gait shuffles and drifts sideways.
- The perception computer's clock is skewed: file timestamps are meaningless.
- **Metal-grid fences fool the depth camera** on its default preset (phantom points
  0.4-0.7 m ahead on the bridge, 2026-10-28). `env/camera.launch.py` uses the High
  Accuracy preset (3) for that; don't change it back.

## Known limitations

- **Software e-stop only.** The handheld and the power switch are the only
  hardware stops.
- **Blind spots.** The depth camera sees ±45° ahead only. Walking backwards
  with `walk()` and all of `teleop.py` have no obstacle guard. The lidar
  sees round him, but not low things right beside him.
- **Nav2 needs a floor map** (docs/navigation.md), and the Orin: without it he walks,
  turns and uses the depth camera, but does not navigate or speak (his voice is on the Orin too).
- **`estop(disarm=True)`** while standing is untested.
- Of the built-in tricks, `long_jump` and `turn_over` have never been run on this robot.
