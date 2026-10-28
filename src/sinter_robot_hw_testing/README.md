# sinter_robot_hw_testing

This package contains a MoveIt Pro robot configuration.

The real space sinter arm with a simulated sun camera, so sun tracking algorithms can be
tested on the actual hardware indoors.

## Usage

```bash
moveit_pro run -c sinter_robot_hw_testing
```

## What's real and what's simulated

Everything comes from `sinter_robot_hw` (physical Kinova driver, controllers, objectives,
RealSense) except the sun camera. The Arducam driver is not started; instead
`sim_sun_camera` from `sinter_robot_mock` publishes `/camera/image_raw` and
`/camera/camera_info`. It renders the sun from the pose of `ee_camera_optical_link`, which
here comes from the real arm's joint states, so moving the real arm moves the blob just as
it would outdoors.

`TrackBlob`, `GetSunOrientation` and the other camera objectives run unchanged, and the
`Reset Sun To Camera` objective (from `sinter_robot_mock`) is available to put the sun back
in view. The sun parameters are in `sinter_robot_mock/config/sim_sun_camera.yaml`; see the
`sinter_robot_mock` README for the commands to move it or make it drift, e.g.

```bash
ros2 service call /sim_sun_camera/place_sun_in_view std_srvs/srv/Trigger
ros2 param set /sim_sun_camera sun_drift_deg_per_s 0.5
```

The arm really moves, so keep the workspace clear: a drifting or misplaced sun will drive it
wherever the tracker goes.
