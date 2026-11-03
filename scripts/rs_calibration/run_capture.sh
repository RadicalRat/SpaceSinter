#!/usr/bin/env bash
# Capture RealSense snapshots for the mount calibration while "Calibrate RealSense Mount" runs.
#   scripts/rs_calibration/run_capture.sh [OUT_DIR]    (default ~/sinter_bags/rs_cal_<timestamp>)
# Ctrl-C (or `docker stop sinter-rs-cal`) when the objective is done, then:
#   python3 scripts/rs_calibration/solve.py OUT_DIR
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="${1:-$HOME/sinter_bags/rs_cal_$(date +%Y-%m-%d_%H%M%S)}"
mkdir -p "$out"
dds="$out/.cyclonedds.xml"
docker exec moveit_pro-agent_bridge-1 bash -c 'cat "$CYCLONEDDS_URI" 2>/dev/null || cat /home/*/.ros/cyclonedds.xml' >"$dds"
echo "capturing into $out"
exec docker run --rm --init --name sinter-rs-cal --network host -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=/cyclonedds.xml \
  -v "$dds:/cyclonedds.xml:ro" -v "$out:/out" -v "$here:/rs_cal:ro" --entrypoint bash sinter-recorder:latest \
  -c "source /opt/ros/humble/setup.bash && exec python3 -u /rs_cal/capture.py /out"
