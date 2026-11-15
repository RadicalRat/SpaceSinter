#!/usr/bin/env bash
# Run a MoveIt Pro objective by name from outside the stack and record the tracking topics.
#   scripts/tracking/run_objective.sh "Track Raster With Sun Mock" [out.npz]
# Joins the running stack's ROS graph from a throwaway agent-bridge container (same DDS config).
set -euo pipefail
NAME="$1"; OUT="${2:-track_$(date +%H%M%S).npz}"; CANCEL="${3:-}"
HERE="$(cd "$(dirname "$0")" && pwd)"; WORK="$(mktemp -d)"
docker exec moveit_pro-agent_bridge-1 bash -c 'cat "$CYCLONEDDS_URI" 2>/dev/null || cat /home/*/.ros/cyclonedds.xml' >"$WORK/cyclonedds.xml"
cp "$HERE/run_objective.py" "$WORK/"
docker run --rm --network host --entrypoint bash -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=/cyclonedds.xml \
  -v "$WORK/cyclonedds.xml:/cyclonedds.xml:ro" -v "$WORK:/work" \
  moveit-studio-agent-bridge:9.4.4-humble-KinovaSinter \
  -c "source /opt/overlay_ws/install/setup.bash && python3 /work/run_objective.py \"$NAME\" /work/out.npz $CANCEL"
cp "$WORK/out.npz" "$OUT" && echo "saved $OUT"
