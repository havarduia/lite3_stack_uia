"""protocol - the stack's addresses and the panel's password, in one place. No ROS.

The robot's own codes, addresses and send() are the SDK's (lite3.udp); the
names the stack used from here are passed through. Send one raw code from the
shell - no ROS, and NO checks: stand_toggle lies him down if he is already
standing, and nothing checks posture before a trick:

    python3 -m robot.protocol stand_toggle
    python3 -m robot.protocol hello            # a name from ACTIONS
    python3 -m robot.protocol 0x21010300       # any number
    python3 -m robot.protocol stick_pitch -16000
    python3 -m robot.protocol camera [on|off]   # depth camera service only
    python3 -m robot.protocol voa [on|off]      # voa (brings the camera too)
"""

from lite3 import udp
from lite3.depth import camera, service
from lite3.udp import ACTIONS, MOTION_ADDR, MOTION_IP, send  # noqa: F401

AP_IP = '192.168.2.1'                # the motion computer on its own wifi (wifi_mode.sh)
ORIN_IP = '192.168.1.5'              # the Orin: lidar, its Nav2, his voice (tts_server.py)
# jetson2app on THIS (perception) computer: the app/handheld AI switch
# APP_ADDR = ('127.0.0.1', 43899)
# The same mediamtx also serves that stream as WebRTC: signalling (WHEP) over
# HTTP, then all the video over one UDP port.
CAMERA_WHEP = 'http://%s:8889/test/whep' % MOTION_IP
# rs_stream.py publishes the RealSense colour image to the same mediamtx.
RS_RTSP = 'rtsp://%s:8554/realsense' % MOTION_IP
RS_WHEP = 'http://%s:8889/realsense/whep' % MOTION_IP
CAMERA_RTC_PORT = 8189
AP_RTC_PORT = 8190                   # the motion computer forwards it to udp_relay's 8189 (env/robot-nat.sh)

# --- app port (APP_ADDR): the app's AI switch, no longer used ---
# AI_SERVICES = 0x21012109    # value 0x40 start, 0x00 stop
# AI_QUERY = 0x2101210D       # reply: same code, value 0x11 up / 0x10 down
# CAMERA_ON, CAMERA_OFF = 0x40, 0x00

def voa(on=None):
    """voa_ros2. Its drop-in (env/voa_ros2-override.conf) binds it to the
    camera: starting voa starts the camera, stopping the camera stops voa."""
    return service('voa_ros2.service', on)


def lookup(name):
    """A code constant in lite3.udp (any case), an ACTIONS name, or a number."""
    if name.lower() in ACTIONS:
        return ACTIONS[name.lower()][0]
    code = getattr(udp, name.upper(), None)
    return code if isinstance(code, int) else int(name, 0)


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    if sys.argv[1] in ('camera', 'voa'):     # camera|voa [on|off]
        on = {'on': True, 'off': False}.get(sys.argv[2] if len(sys.argv) > 2 else None)
        globals()[sys.argv[1]](on)
        sys.exit(print('camera: %s   voa: %s' % tuple(
            'up' if f() else 'down' for f in (camera, voa))))
    code = lookup(sys.argv[1])
    value = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0
    send(code, value)
    print('sent 0x%08X value=%d -> %s:%d' % ((code, value) + MOTION_ADDR))
