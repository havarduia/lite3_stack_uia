#!/bin/bash
# Launch Nav2 on a floor map: start_nav2_map.sh <floor>. Normally called by
# Lite3.nav_start(floor), which enforces stand-first ordering and cleans up
# orphaned nodes. The floor's map is ~/lite3_maps/<floor>/map.yaml (env/get_map.sh).
ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
source "$ENV_DIR/lite3_env.sh"
FLOOR="${1:?usage: start_nav2_map.sh <floor>}"
# Nav2 Humble on the Orin plans and drives (orin/bin/nav.sh: Smac + MPPI + collision monitor),
# through robot/orin_nav.py; the Orin's 3D localizer (robot/cloud_locate.py) places him on the map.
# Here: the map, the costmaps and the planner, for the panel and the checks before a goal.
# ORIN_NAV_DRY=1: everything runs but nothing reaches /cmd_vel (a bench check).
MAP="$HOME/lite3_maps/$FLOOR/map.yaml"
[ -f "$MAP" ] || { echo "no map for floor '$FLOOR' ($MAP)" >&2; exit 1; }
# The parameters are the vendor's dr_nav2_mapless file (our patch's tuning) with the map added,
# made afresh each start so that file stays the only copy.
SHARE=$(python3 -c "from ament_index_python.packages import get_package_share_directory as d; print(d('dr_nav2_mapless'))")
PARAMS=/tmp/nav2_map.yaml
python3 "$ENV_DIR/nav2_map_params.py" "$SHARE/config/lite_nav2_mapless.yaml" "$MAP" "$PARAMS" || exit 1
# The sonars for the range layer, and "is he localized" (robot/locate.py). Same
# process group as the launch below, so nav_stop() kills them with the stack.
rm -f /tmp/sonar_range.ready
python3 -m robot.sonar_range &
CLOUD=$(cat "$HOME/lite3_maps/$FLOOR/cloud.txt" 2>/dev/null) \
    || { echo "floor '$FLOOR' has no cloud.txt (its walk on the Orin): the 3D localizer needs one" >&2; exit 1; }
python3 -m robot.lio_relay map "$CLOUD"
python3 -m robot.locate "$FLOOR" &
ORIN_SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=2 -o ServerAliveCountMax=3 lite3@192.168.1.5"
# the floor's map, as this Nav2 has it, to the Orin's
$ORIN_SSH "mkdir -p lite3_maps/$FLOOR" \
    && scp -q -o BatchMode=yes "$HOME/lite3_maps/$FLOOR/map.yaml" "$HOME/lite3_maps/$FLOOR/map.pgm" \
           "lite3@192.168.1.5:lite3_maps/$FLOOR/" \
    || { echo "could not copy the map to the Orin" >&2; exit 1; }
# runs while this ssh does: nav_stop() ends it with the group, and the Orin's stops (orin/bin/nav.sh)
sleep infinity | $ORIN_SSH "~/bin/nav.sh run $FLOOR" &
python3 -m robot.orin_nav ${ORIN_NAV_DRY:+dry} &
for _ in $(seq 100); do [ -e /tmp/sonar_range.ready ] && break; sleep 0.1; done
exec ros2 launch "$ENV_DIR/nav2_map.launch.py" params:="$PARAMS"
