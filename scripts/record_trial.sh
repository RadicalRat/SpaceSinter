#!/bin/bash
# Record a sintering trial to a rosbag (internal disk for now, see BAG_ROOT below).
#
#   scripts/record_trial.sh [options] [label]   start recording (Ctrl-C stops it), then start the trial
#   scripts/record_trial.sh stop                stop a recording started with --detach
#   scripts/record_trial.sh status              show whether a recording is running
#
# Options:
#   -d, --detach   record in the background; stop it later with `record_trial.sh stop`
#   --depth        also record the RealSense point cloud (large)
#   --rebuild      rebuild the recorder image first
#
# Each run writes <SINTER_BAG_DIR>/<YYYY-MM-DD_HHMMSS>_<label>/ (an MCAP bag, zstd-compressed) plus a
# <same name>.trial.yaml metadata sheet to fill in (cloud cover, wind, simulant, ...). The MoveIt Pro stack must
# be running; the recorder joins its ROS graph from its own container, so nothing in the stack is touched.
set -euo pipefail

# For now bags go to the internal disk (one USB port short for the Seagate); move them to
# "/media/cameron/Seagate Portable Drive/sinter_bags" after a trial. To record straight to the Seagate again, set
# SINTER_BAG_DRIVE="/media/cameron/Seagate Portable Drive" SINTER_BAG_DIR="$SINTER_BAG_DRIVE/sinter_bags".
BAG_ROOT="${SINTER_BAG_DIR:-$HOME/sinter_bags}"
DRIVE="${SINTER_BAG_DRIVE:-/}"
STACK_CONTAINER=moveit_pro-agent_bridge-1
RECORDER=sinter_recorder
IMAGE=sinter-recorder:latest
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Logging list from paper outline v2 section 5, mapped to this stack's topics.
TOPICS=(
  /joint_states /tf /tf_static /robot_description
  /joint_trajectory_admittance_controller/controller_state /trajectory_execution_event
  /behavior_tree_status /failure_reason_collector /rosout
  /sinter/sun_error /sinter/raster_path /sinter/bed_plane /sinter/bed_tilt_deg
  /sinter/sun_ephemeris /sinter/sun_azimuth_deg /sinter/sun_elevation_deg /sinter/bed_incidence_deg
  /srrl/dni_w_m2 /srrl/ghi_w_m2 /srrl/dhi_w_m2 /srrl/irradiance
  /camera/camera_info /camera/image_raw/compressed
)

die() { echo "record_trial: $*" >&2; exit 1; }
running() { [ -n "$(docker ps -q -f name="^${RECORDER}$")" ]; }

case "${1:-}" in
  stop)
    running || die "no recording is running"
    bag=$(docker inspect -f '{{index .Config.Labels "sinter.bag"}}' "$RECORDER")
    echo "Stopping recording $bag ..."
    # SIGINT lets ros2 bag close the file cleanly; a hard kill leaves an unreadable bag.
    docker kill -s INT "$RECORDER" >/dev/null
    docker wait "$RECORDER" >/dev/null 2>&1 || true
    echo "Saved $bag"
    exit 0 ;;
  status)
    if running; then
      echo "Recording: $(docker inspect -f '{{index .Config.Labels "sinter.bag"}}' "$RECORDER")"
    else
      echo "Not recording"
    fi
    exit 0 ;;
esac

detach=false depth=false rebuild=false label=""
while [ $# -gt 0 ]; do
  case "$1" in
    -d|--detach) detach=true ;;
    --depth) depth=true ;;
    --rebuild) rebuild=true ;;
    -h|--help) sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) die "unknown option $1 (see --help)" ;;
    *) label="$1" ;;
  esac
  shift
done

# A missing drive would otherwise silently fill the internal disk.
mountpoint -q "$DRIVE" || die "$DRIVE is not mounted; plug in the drive (or set SINTER_BAG_DRIVE / SINTER_BAG_DIR)"
mkdir -p "$BAG_ROOT"
free_gb=$(df -BG --output=avail "$BAG_ROOT" | tail -1 | tr -dc 0-9)
[ "$free_gb" -ge 5 ] || die "only ${free_gb} GB free at $BAG_ROOT; move old bags off first"
echo "${free_gb} GB free at $BAG_ROOT (about $((free_gb * 1000 / 7200)) h at 2 MB/s; much less with --depth)"
running && die "already recording ($(docker inspect -f '{{index .Config.Labels "sinter.bag"}}' "$RECORDER")); run '$0 stop' first"
[ -n "$(docker ps -q -f name="^${STACK_CONTAINER}$")" ] || die "MoveIt Pro is not running ($STACK_CONTAINER not found)"

if $rebuild || [ -z "$(docker images -q "$IMAGE")" ]; then
  echo "Building $IMAGE (one time) ..."
  base=$(docker inspect -f '{{.Config.Image}}' "$STACK_CONTAINER")
  docker build -q -t "$IMAGE" --build-arg BASE_IMAGE="$base" -f "$REPO_DIR/scripts/recorder.Dockerfile" "$REPO_DIR/scripts" >/dev/null
fi

name="$(date +%Y-%m-%d_%H%M%S)"
[ -n "$label" ] && name="${name}_${label// /_}"
mkdir -p "$BAG_ROOT"

# Use the stack's own DDS config so the recorder discovers its nodes (loopback only, no multicast).
dds_config="$BAG_ROOT/.cyclonedds.xml"
docker exec "$STACK_CONTAINER" bash -c 'cat "$CYCLONEDDS_URI" 2>/dev/null || cat /home/*/.ros/cyclonedds.xml' >"$dds_config"

topics=("${TOPICS[@]}")
$depth && topics+=(/camera/camera/depth/color/points)

# Metadata sheet (v2 section 5). Fill in the blanks after the trial.
objective="$REPO_DIR/src/sinter_robot_hw/objectives/sinter_raster_closed_loop.xml"
tunables=$(grep -o 'code="line_spacing_m[^"]*"' "$objective" 2>/dev/null | sed 's/^code="//; s/"$//' || true)
cat >"$BAG_ROOT/$name.trial.yaml" <<EOF
bag: $name
label: "$label"
start_local: "$(date --iso-8601=seconds)"
start_utc: "$(date -u --iso-8601=seconds)"
git_commit: "$(git -C "$REPO_DIR" rev-parse --short HEAD 2>/dev/null)$(git -C "$REPO_DIR" diff --quiet 2>/dev/null || echo '-dirty')"
objective_tunables: "$tunables"
condition_id: ""
result: ""          # success / failed (keep failed runs, labeled)
cloud_cover: ""
wind: ""
simulant: ""
bed_geometry: ""
base_level_pitch_roll_deg: ""
base_heading_deg: ""
notes: ""
EOF

run_flags=(--rm --init --name "$RECORDER" --label "sinter.bag=$BAG_ROOT/$name" --network host
           -u "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
           -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=/cyclonedds.xml
           -v "$dds_config:/cyclonedds.xml:ro" -v "$BAG_ROOT:/bags" --entrypoint bash)
if $detach; then
  run_flags+=(-d)
elif [ -t 0 ]; then
  run_flags+=(-it)
fi

echo "Recording ${#topics[@]} topics to $BAG_ROOT/$name"
$detach || echo "Start the trial now. Press Ctrl-C to stop recording."
# Ctrl-C reaches ros2 bag (which closes the bag cleanly); keep this script alive to print where it went.
trap 'true' INT
docker run "${run_flags[@]}" "$IMAGE" -c \
  "source /opt/overlay_ws/install/setup.bash && exec ros2 bag record -o /bags/$name -s mcap --storage-preset-profile zstd_fast ${topics[*]}" \
  || true

if $detach; then
  echo "Recording in the background. Stop it with: $0 stop"
else
  echo "Saved $BAG_ROOT/$name (fill in $name.trial.yaml)"
fi
