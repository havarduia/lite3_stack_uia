# Background: design principles, FAQ, history, glossary

## Design principles

These are the ideas behind the code; useful if asked "why is it built like this?"

1. **Turn silent failures into loud ones.** The robot's worst habit is
   ignoring commands without saying why. Every known case is either handled
   automatically (auto mode, posture mode exit, swallowed arming toggle) or
   raises `Lite3Error` with a message that says what to do. `demo.py` lesson 4
   calls the refusals "the docs".
2. **Guarantee the stop.** Every motion call ends in `finally: halt()`. The
   e-stop kills the *other* publisher (Nav2), not just our own. Remote driving
   uses dead-man timeouts, and a Go from the web panel is cancelled when no
   page is left to stop it. Shutdown order is always "stop moving, then
   disarm".
3. **State as attributes, not callbacks.** A background spin thread keeps the
   latest message of each topic, so behaviours read like ordinary sequential
   Python.
4. **Blocking calls that report what happened.** `walk()` returns how far it
   actually moved and why it stopped, because stopping short for an obstacle
   is normal operation.
5. **Measure, don't guess.** Constants come from tape measures, camera checks
   and logs; the docs record the measurement and date.
6. **Keep ROS optional where possible.** `protocol.py`, `voice.py` and the
   SDK's `udp`, `speaker` and `camera` don't import `rclpy`; `lite3.Lite3` loads it only when used.
7. **Change vendor code minimally, and keep the diff.** Every vendor change is
   a patch file in `env/` (or `orin/`) with the reasoning inside.
8. **Navigation is bolted on.** Everything that needs the lidar (FAST-LIO2, the
   3D localizer, Nav2) runs on the Orin we added; perception runs the SDK, the
   panel, the camera driver and relays. Take the Orin away and the robot is close
   to stock. New navigation work goes on the Orin.

## Questions you are likely to be asked

**Why not just use ROS directly?**
You can, but you'd have to rediscover every silent failure: wrong DDS
implementation, not in auto mode, not standing, low battery, posture mode,
unarmed robot, Nav2 outliving your process. `Lite3` encodes all of that once.

**How does it avoid obstacles?**
Two layers. (1) `walk()`/`turn()` check the depth camera at 4 Hz and stop.
(2) Nav2 on the Orin, on a floor map, plans around obstacles in a costmap
built from the lidar and the depth camera (3D voxels that fade after 2.5 s), and
its collision monitor checks every command against the lidar, the camera and the
front sonar ([navigation.md](navigation.md#what-runs-where)). The vendor's `voa`
node could correct velocities too (we fixed it so it actually can), but it is off.

**How do you convert the point cloud into obstacles?**
Transform from the camera's optical frame to the robot body frame, correcting
for the 20° downward tilt; keep points 8–60 cm above the floor; bin by bearing;
take the nearest per bin. "In the path" is judged by lateral offset against
the robot's half-width.

**How accurate is odometry?**
Raw leg odometry under-counted by 15%; after scaling, walks agree with tape.
Turns are accurate to about ±1° after coast compensation. Leg odometry
alone drifts over long runs; since 2026-10-08 FAST-LIO2 on the Orin corrects
it into `/odom_fused` (a 10.4 m walked loop closed to 0.05 m, against 0.21 m
by the legs alone; [lidar.md](lidar.md)).

**What happens if the program crashes or Wi-Fi drops?**
Motion calls always publish zero velocity on exit. Remote driving (teleop,
web) uses dead-man timeouts of 0.3–0.5 s, and a Go started from the web panel
is cancelled 10 s after its last page disconnects. Beyond that, the robot's
own failsafe stops it ~1.5 s after velocity commands stop arriving.

**Why does the robot talk only when standing still?**
The motors are too loud to hear the speaker while walking.

**What did you change in vendor code?**
Four in `env/` and FAST-LIO2's in `orin/` ([Patches (env/*.patch)](environment.md#patches-envpatch)); the most important fixed a bug where every velocity
command reached the robot twice, which made the vendor obstacle avoider
useless, and one where its configuration was silently never loaded.

## History

`git log`, and the lab notes in [docs/history/](history/README.md).

## Glossary

| Term | Meaning |
|---|---|
| **jy_exe** | the vendor's closed-source locomotion controller on the motion computer |
| **basic_state** | `jy_exe`'s posture/state number (6 = standing; see `docs/robot.md` in the SDK) |
| **heartbeat / interlock** | keepalive packet without which `jy_exe` ignores commands |
| **transfer_ros2 / Jetson2Motion** | vendor bridge between the robot's UDP protocol and ROS2 |
| **voa** | vendor "visual obstacle avoidance" node: corrects `/cmd_vel` → `/cmd_vel_corrected` |
| **Twist / `/cmd_vel`** | ROS velocity message / topic: `linear.x` forward, `linear.y` left, `angular.z` turn |
| **odom frame** | coordinate frame fixed where odometry started; drifts over time |
| **base_link** | coordinate frame attached to the robot body centre |
| **TF** | ROS's system for transforms between frames |
| **Nav2** | the standard ROS2 navigation stack |
| **costmap** | grid of traversal costs (0 free … 99 lethal) Nav2 plans on |
| **STVL** | Spatio-Temporal Voxel Layer: costmap layer of decaying 3D voxels |
| **inflation** | costmap layer that adds cost near obstacles, keeping the robot away from them |
| **Smac 2D / MPPI** | Nav2's global planner / its local controller (Model Predictive Path Integral), both on the Orin since 2026-11-24 |
| **collision monitor** | Nav2 node with the last word on every command: stops or slows him for what the sensors see in its zones |
| **FAST-LIO2** | lidar-inertial odometry on the Orin; corrects the leg odometry |
| **Orin** | the Orin Nano we added, with the lidar: all the navigation runs there |
| **CycloneDDS** | the DDS middleware ROS2 uses here (the default FastRTPS doesn't work on this robot) |
| **RMW** | ROS middleware interface; selects the DDS implementation |
| **Kokoro** | the neural text-to-speech on the Orin that is his voice (Piper before 2026-11-09) |
| **MJPEG** | video as a stream of JPEG images; plays in any `<img>` tag |
| **Tailscale** | VPN; the HMI is reachable through it, or on his own wifi |
| **dead-man switch** | control that stops the machine when the operator stops actively holding it |
