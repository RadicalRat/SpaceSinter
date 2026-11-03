# Image for scripts/record_trial.sh: the MoveIt Pro agent image (so the MoveIt Pro message types can be recorded)
# plus rosbag2 with MCAP storage. Built by record_trial.sh on first use; rebuild with `record_trial.sh --rebuild`.
ARG BASE_IMAGE=moveit-studio-agent-bridge:9.4.4-humble-KinovaSinter
FROM ${BASE_IMAGE}
USER root
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends ros-humble-ros2bag ros-humble-rosbag2-storage-mcap && \
    rm -rf /var/lib/apt/lists/*
