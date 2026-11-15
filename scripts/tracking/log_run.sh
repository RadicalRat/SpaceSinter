#!/usr/bin/env bash
# Passive logger (no goals sent): scripts/tracking/log_run.sh OUT.npz [TIMEOUT_S]
set -euo pipefail
OUT="$(realpath -m "$1")"; TO="${2:-1500}"; HERE="$(cd "$(dirname "$0")" && pwd)"; WORK="$(mktemp -d)"
docker exec moveit_pro-agent_bridge-1 bash -c 'cat "$CYCLONEDDS_URI" 2>/dev/null || cat /home/*/.ros/cyclonedds.xml' >"$WORK/c.xml"
cp "$HERE/log_run.py" "$WORK/"
docker run --rm --network host --entrypoint bash -u "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONUNBUFFERED=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=/c.xml -v "$WORK/c.xml:/c.xml:ro" -v "$WORK:/work" \
  moveit-studio-agent-bridge:9.4.4-humble-KinovaSinter -c "source /opt/overlay_ws/install/setup.bash && python3 /work/log_run.py /work/out.npz $TO"
cp "$WORK/out.npz" "$OUT"; cp "$WORK/out.log.txt" "${OUT%.npz}.log.txt"; echo "saved $OUT"
