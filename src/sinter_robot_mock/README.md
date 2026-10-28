# sinter_robot_mock

This package contains a MoveIt Pro robot configuration.

mock hardware configuration for the solar sinter kinvoa arm

## Usage

This robot configuration package can be used with MoveIt Pro by specifying it as the configuration package:

```bash
moveit_pro run -c sinter_robot_mock
```

## Simulated sun camera

The mock config starts `sim_sun_camera` (see `launch/sim_nodes.launch.py`), which stands in
for the real Arducam on `/camera/image_raw` and `/camera/camera_info`. It renders the sun as a
bright blob, projected through the real camera calibration from the current pose of
`ee_camera_optical_link`, so moving the mock arm moves the blob just like the real camera.
`TrackBlob`, `GetSunOrientation` and the other camera objectives from `sinter_robot_hw`
are loaded in the mock config and run against it unchanged.

The sun starts slightly off the camera axis so there is always something to track. Useful
commands (all parameters are in `config/sim_sun_camera.yaml`):

```bash
# Put the sun back just off the current camera axis
ros2 service call /sim_sun_camera/place_sun_in_view std_srvs/srv/Trigger
# Move the sun to a fixed direction (world frame)
ros2 param set /sim_sun_camera sun_azimuth_deg 30.0
ros2 param set /sim_sun_camera sun_elevation_deg 60.0
# Let the sun drift to test continuous tracking
ros2 param set /sim_sun_camera sun_drift_deg_per_s 0.5
```

The sun is also published as the TF frame `sim_sun` for visualization.
