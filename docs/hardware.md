# Hardware: the computers and how a command reaches the legs

The stock robot alone is described in the SDK's `docs/robot.md`; this page
adds what this project put around it (the Orin, Nav2, the panel).

## The hardware and the three computers

The Lite3 Pro comes with a "split brain": an RK3588 ARM motion host running the
high-rate (1 kHz) joint control loop, and a Jetson Xavier NX perception host
for vision ([DEEP Robotics](https://www.deeprobotics.us/products/lite-3/)).
This project bolted a third on: an Orin Nano with a Livox MID-360 lidar, on an
Ethernet switch on his back. **All the navigation runs on the Orin**; perception
only relays to it, so taking the Orin away leaves the robot close to stock.

```
                        ┌───────────────────────────────────────────┐
  Handheld remote ─────►│  MOTION computer  (RK3588, 192.168.1.120) │
  (sticks, heartbeat)   │                                           │
                        │  jy_exe      the vendor locomotion        │
                        │              controller. UDP :43893 in,   │
                        │              telemetry out to :43897      │
                        │  front wide-angle camera → RTSP :8554     │
                        │  speaker (ES8388 codec), played by aplay  │
                        │  front + rear ultrasonic sensors          │
                        └──────────────────┬────────────────────────┘
                                   Ethernet │ UDP
                        ┌──────────────────▼────────────────────────┐
                        │  PERCEPTION computer (Jetson Xavier NX)   │
                        │  Ubuntu 20.04, ROS2 Foxy, CycloneDDS      │
                        │                                           │
  RealSense D435i ─USB─►│  realsense_ros2.service  depth + colour   │
  (depth camera,        │  transfer_ros2.service   UDP <-> ROS2     │
   pitched 20° down)    │         (Jetson2Motion, patched by us)    │
                        │  ► THIS REPOSITORY ◄  lite3_sdk +         │
                        │    StackRobot, the web panel (hmi), the   │
                        │    relays to the Orin (orin_nav.py,       │
                        │    lio_relay.py, sonar_range.py)          │
                        └──────────────────┬────────────────────────┘
                                   Ethernet │ UDP 8044/8045 (and more)
                        ┌──────────────────▼────────────────────────┐
  Livox MID-360 ─Eth──► │  ORIN Nano (192.168.1.5), added by us     │
  (192.168.1.102)       │  Ubuntu 22.04, ROS2 Humble                │
                        │  livox driver, FAST-LIO2 (odometry),      │
                        │  cloud_locate (where he is on the map),   │
                        │  Nav2: Smac 2D, MPPI, collision monitor;  │
                        │  local costmap = lidar + depth camera     │
                        │  Kokoro TTS (his voice), lidar panel :8000│
                        └───────────────────────────────────────────┘
          laptop / phone ── Tailscale (VPN) ──► perception's web HMI :8080
```

The vendor's obstacle avoider (`voa_ros2`) is off: the panel's switch for it stays off,
and the Orin's collision monitor does its job.

Where each sensor lives matters in the code:

| Thing | Lives on | How our code reaches it |
|---|---|---|
| Walking, posture, tricks | motion (`jy_exe`) | UDP packets from the SDK (`lite3.udp`), or `/cmd_vel` via `transfer_ros2` |
| Robot state, battery, leg odometry, IMU, sonars | motion → republished on perception | ROS2 topics from `transfer_ros2`; the sonars go on to the Orin |
| Depth camera (RealSense D435i) | perception, USB | ROS2 topic `/camera/depth/color/points`; thinned copies go to the Orin's Nav2 |
| Lidar (Livox MID-360) | Orin, Ethernet | FAST-LIO2's pose comes back as `/odom_fused` (`robot/lio_relay.py`) |
| Navigation (Nav2) | Orin | goals and the speed it drives at pass through `robot/orin_nav.py` |
| Front wide-angle camera | motion | RTSP stream, grabbed with `ffmpeg` |
| Speaker | motion | `ssh` to the motion computer, run `aplay` |

## How a command reaches the legs

There are **two paths** into `jy_exe`, and the code uses both.

### Path A: "simple" UDP frames (posture, mode, heartbeat, tricks)

The SDK's `lite3/udp.py` sends 12-byte packets `<code:uint32, value:int32, 0>`
directly to `192.168.1.120:43893`. Examples:

| Code | Meaning |
|---|---|
| `0x21010202` | stand ↔ lie. **A toggle**: sending it to a standing robot lies it down |
| `0x21010C03` | auto mode. Without it, velocity commands are silently ignored |
| `0x21010C02` | manual mode (the handheld sends this when a stick moves) |
| `0x21040001` | heartbeat. The controller keepalive, needed ≥ 2 Hz |
| `0x21010D05` / `0D06` | enter / leave posture ("twist body") mode |
| `0x21010130` | pitch stick (in posture mode: tilts the body) |
| `0x21010506` etc. | built-in tricks (`hello`, `dance`, …) |

Some codes come from DEEP Robotics' official source, some from a community
SDK, and several were captured from the handheld and phone app by us.
`lite3.udp.SOURCES` records which is which (the SDK's `docs/robot.md`).

### Path B: ROS2 velocity (`/cmd_vel`)

```
our code ──/cmd_vel (Twist)──► Jetson2Motion (transfer_ros2) ──UDP cmd 320──► jy_exe
the Orin's Nav2 ──UDP──► robot/orin_nav.py ──/cmd_vel──┘
```

`Jetson2Motion` is the vendor's bridge node. In the other direction it takes
the robot's UDP telemetry (port 43897) and publishes it as ROS topics:

| Topic | Contents |
|---|---|
| `/robot_state_debug` | `Int32MultiArray`: basic_state, gait, …, **battery** (we added battery, error, charging) |
| `/leg_odom2` | odometry from leg kinematics (we scale it by 1.15) |
| `/imu/data` | body IMU |
| `/us_publisher/ultrasound_distance` | rear sonar |
| `/us_publisher/ultrasound_front` | front sonar (we added this; stock only published one) |
| `/handle_state` | the handheld's stick positions |

### The interlock ("arming")

`jy_exe` ignores every network command unless a controller heartbeat keeps
arriving. Normally the handheld sends it. `Lite3.heartbeat_start()` sends it
from Python instead (2 Hz), which means the robot obeys **with no handheld
powered on**. That is convenient and also the most important safety caveat in
the project: the terminal (Ctrl-C) becomes your stop button.

Even with the heartbeat, a freshly powered robot is "settled but not armed":
the **first real command is swallowed** by `jy_exe`'s own arming sequence.
`stand()` detects this and re-sends the toggle (the SDK's `docs/robot.md`). Measured
2026-09-21: the heartbeat alone does not arm him, and neither do mode
commands; he sits at 98 indefinitely until a real command arrives, so no
amount of waiting after `heartbeat_start()` helps.
