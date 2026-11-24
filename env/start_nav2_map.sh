#!/bin/bash
# Nav2 on a floor map: start_nav2_map.sh <floor>. Normally called by
# Lite3.nav_start(floor), which enforces stand-first ordering and cleans up
# orphaned nodes. The floor's map is ~/lite3_maps/<floor>/map.yaml (env/get_map.sh).
ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
source "$ENV_DIR/lite3_env.sh"
FLOOR="${1:?usage: start_nav2_map.sh <floor>}"
# Nav2 Humble on the Orin plans and drives (orin/bin/nav.sh: Smac + MPPI + collision monitor),
# through robot/orin_nav.py; the Orin's 3D localizer (robot/cloud_locate.py) places him on the map.
# Here no Nav2 of its own (2026-11-24): the sonars, "is he localized" and the link to the Orin.
# ORIN_NAV_DRY=1: everything runs but nothing reaches /cmd_vel (a bench check).
MAP="$HOME/lite3_maps/$FLOOR/map.yaml"
[ -f "$MAP" ] || { echo "no map for floor '$FLOOR' ($MAP)" >&2; exit 1; }
# The sonars (for the Orin's collision monitor) and "is he localized" (robot/locate.py). One
# process group, so nav_stop() kills them all.
python3 -m robot.sonar_range &
CLOUD=$(cat "$HOME/lite3_maps/$FLOOR/cloud.txt" 2>/dev/null) \
    || { echo "floor '$FLOOR' has no cloud.txt (its walk on the Orin): the 3D localizer needs one" >&2; exit 1; }
python3 -m robot.lio_relay map "$CLOUD"
python3 -m robot.locate "$FLOOR" &
ORIN_SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=2 -o ServerAliveCountMax=3 lite3@192.168.1.5"
# the floor's map to the Orin's Nav2
$ORIN_SSH "mkdir -p lite3_maps/$FLOOR" \
    && scp -q -o BatchMode=yes "$HOME/lite3_maps/$FLOOR/map.yaml" "$HOME/lite3_maps/$FLOOR/map.pgm" \
           "lite3@192.168.1.5:lite3_maps/$FLOOR/" \
    || { echo "could not copy the map to the Orin" >&2; exit 1; }
# runs while this ssh does: nav_stop() ends it with the group, and the Orin's stops (orin/bin/nav.sh)
sleep infinity | $ORIN_SSH "~/bin/nav.sh run $FLOOR" &
python3 -m robot.orin_nav ${ORIN_NAV_DRY:+dry} &
wait
