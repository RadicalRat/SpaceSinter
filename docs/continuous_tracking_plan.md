# Continuous focal-path + sun tracking (plan and log)

Started 2026-11-11 after the advisor review. Goal: replace the discrete scan loop (at each waypoint: check the sun,
re-align if over tolerance, plan a Cartesian move to the next waypoint, execute, stop) with continuous control:

- 3 translational DOF: the focal point (`ee_focal_point_link`) follows the raster path at a set speed.
- 3 rotational DOF: the lens axis tracks the sun continuously from the sun sensor; roll about the optical axis is held.

This is the continuous form of the outline's formulation (lens axis ∥ ŝ(t), F = p(t)).

## Why

- Trial 11 (check every 4 mm) still gives a sawtooth: median 0.09°, p95 0.18°. The disturbances are slow (sun
  0.004 °/s + deflection 0.006 °/mm × ~1 mm/s ≈ 0.01 °/s), so a proportional loop with gain K ≈ 0.5–1 /s should hold
  about 0.01–0.02° steady state, near the sensor noise floor (0.007°/frame).
- R6 becomes a bandwidth requirement (steady error = disturbance rate / gain) instead of a re-check schedule.
- Constant scan speed means uniform energy per unit length; the stop-and-go loop gives uneven dwell.

## What the stack already provides (MoveIt Pro 9.4.4 + this repo)

- `velocity_force_controller` (PickNik): a Cartesian twist at `ee_frame` → damped least-squares Jacobian inverse at
  100 Hz. Already configured in `sinter_robot_hw` and `sinter_robot_mock` `ros2_controllers.yaml` with
  `ee_frame: ee_focal_point_link`, inactive at startup. Command: `moveit_pro_controllers_msgs/VelocityForceCommand` on
  `/velocity_force_controller/command`. `control_frame_id` can be `world` (the URDF has a `world` link). Twist
  rejected if older than `command_timeout` (0.2 s). It does **no collision checking** (joint limits only).
- The Kinova driver always streams joint velocities to the JACO2 (position mode = P loop on velocity), so velocity
  control is native.
- Behaviors: `ActivateControllers` / `SwitchController`, `PublishVelocityForceCommand`,
  `ComputeVelocityToAlignWithTarget`, `PoseJog`, `MPCPoseTracking` (MuJoCo MPC; not needed).
- The sun sensor already streams at 28 Hz with the sun detected in 97–99 % of frames (Trials 2 and 11).
- The mock robot (`sinter_robot_mock`) has the same controllers plus `sim_sun_camera.py` (renders the sun from the
  arm's TF), so the whole loop can be developed indoors.

## Control law (one custom behavior, `TrackRasterWithSun`, ~50 Hz timer)

- Rotation: ω_focal = K_rot · (−v_y, +v_x, 0), where (v_x, v_y) are the sun sensor's tan offsets (aim offset removed).
  This is the same sign convention as SunPoseFinder (`setRPY(-vy, vx, 0)` on `ee_focal_point_link`). It is rotated to
  world with the focal frame's TF. ω_z = 0 holds roll.
- Translation: a path parameter s advances at the scan speed along the raster polyline;
  v = V · t̂(s) + K_pos · (p_ref(s) − p_focal), with p_focal from TF (FK).
- Sun lost (no blob): ω = 0 and s stops advancing (no half-melted lines); abort after a timeout.
- Abort (zero twist, FAILURE) on position tracking error > limit or pointing error > limit.
- Halt (BT halt or failure): publish a zero twist; the controller's command timeout also stops the arm.

## Steps

| # | Step | Status |
|---|---|---|
| 0 | Test controller switching (`SwitchController`) in mock, then hardware; the Teleoperate crash happened during a controller switch | mock PASSED 2026-11-11 (objective `Controller Switch Test`); hardware todo |
| 1 | `TrackRasterWithSun` behavior + a mock objective | mock PASSED 2026-11-11 (objective `Track Raster With Sun Mock`) |
| 2 | Restructure the main objective: preflight + planned lead-in → switch → track → switch back → planned lift; failure branch always stops and lifts | todo |
| 3 | Preflight: reachability with the ephemeris orientation at each point's scheduled time; singularity margin | todo |
| 4 | Sim validation: moving sun + deflection term in `sim_sun_camera.py`, show rejection | todo |
| 5 | Hardware tuning: controller limits (now 0.25 m/s, 1 rad/s → a few cm/s, a few °/s), gains, command timeout vs JACO2 ~170 ms USB stalls | todo |

## Risks

- Controller switching may hit the Teleoperate segfault (objective_server, GetParameters client callback). Step 0.
- No collision checking in the velocity controller: rely on the path preflight, the planned lead-in and lift-off,
  and the tracking-error abort.
- Near singularities the damped inverse drifts off the path instead of failing: the tracking-error abort is essential.
- Placement still closes on FK; the open 0.86× travel-scale question is unaffected.

## Log

- 2026-11-11: plan written; started steps 0 and 1.
- 2026-11-11 step 0 (mock): `Controller Switch Test` switched to velocity_force_controller and back via
  `SwitchController` (automatic_deactivation) with no crash; stack stayed healthy. Controller states verified with
  `ros2 control list_controllers`. Hardware run still to do.
- 2026-11-11 mock fix: the fake hardware started at all joints = pi, which is in self-collision
  (ee_end_effector_link <-> link_3), so planned moves failed. `ros2_control_interfaces/config/initial_positions.yaml`
  now starts the mock at BeforeSinter. Mock objectives use link_padding 0.0.
- 2026-11-11 step 1: `TrackRasterWithSun` (src/behaviors/sun_pose_finder/src/track_raster_with_sun.cpp), unit tests in
  test/test_tracking.cpp (path reference, sun-correction sign, clamp). Added an acquire phase: hold the path start
  until the sun error < acquire_tolerance_rad, then scan (the sim sun starts 5 deg off axis).
  Mock run of `Track Raster With Sun Mock` (3 lines x 30 mm, 4 mm apart, 2 mm/s, gains 1/s): acquired in 5.0 s,
  pointing after acquisition median 0.001 deg / p95 0.004 deg (static sim sun, no noise or deflection, so this only
  shows the loop and signs are right), path tracking error median 0.04 mm / p95 0.10 mm / max 0.20 mm (corners),
  measured speed 2.00 mm/s, path complete, trajectory controller restored. Sim sun camera only publishes ~4 Hz
  (Python renderer); the real one is 28 Hz.
- Next: step 4 (moving sun + deflection + noise in sim_sun_camera.py) before hardware; step 0 on hardware.
- 2026-11-11 step 4 (part 1): sim_sun_camera.py now moves the sun about the site's polar axis (real diurnal path,
  latitude 39.7555, north heading 125) by real elapsed time (`sun_drift_deg_per_s`, set to 0.008356 = 2x real),
  adds Gaussian centre jitter (`sun_jitter_px`, 0.5 px ≈ 0.04 deg/axis ≈ 5x real) and publishes ground truth on
  `/sim_sun_camera/true_error`. Run (same objective, gains 1/s, 2 mm/s): acquired in 4.9 s; TRUE pointing error
  median 0.019 deg, p95 0.046, max 0.061; the sensor read median 0.052 / p95 0.10 (the noise). Pure lag would be
  ~0.008 deg; the rest is noise feed-through because the sim camera runs at ~4 Hz, so each noisy frame is applied
  for 0.25 s (≈25 % of each sample's noise reaches the lens; at the real 28 Hz it would be ≈4 %). Path error: a
  0.04 mm saw (TF refresh vs 50 Hz loop: 2 mm/s × 20 ms) and 0.16–0.18 mm spikes at the 4 corners decaying at 1/s.
  Plot: scripts/tracking/track_drift_noise.png. Still to add: a deflection term (pose-dependent tilt, ~0.006 deg/mm).
- 2026-11-11 fault handling (user request: pause on sun loss, stop/restart, a separate recovery objective).
  - TrackRasterWithSun: sun loss now PAUSES (holds focal point + lens, path stops, re-acquires, resumes) and never
    fails by default (`sun_lost_abort_s` and `acquire_timeout_s` default 0 = never). New ports: `start_at_m` (in),
    `progress_m` and `path_world` (out) to resume a stopped run; halt logs where it stopped and sends a zero twist.
  - New objective `Switch To Trajectory Controller` (sinter_robot_hw/objectives, also loaded by the mock): run it
    after stopping a tracking objective from the UI. A cancelled objective cannot switch back itself (verified:
    velocity_force_controller stays active after a cancel).
  - Sim: `sun_hidden` param and `~/hide_sun` service (hides for `hide_duration_s`) in sim_sun_camera.py.
  - Mock tests (all passed): `Test Sun Loss Pause` (two 5 s clouds mid-scan: progress frozen exactly, focal point
    held within 0.07 mm, true pointing error max 0.072 deg over the run, resumed at the same arc length, path
    complete); `Test Stop And Resume` (halted at 18.9 mm via a Timeout decorator, recovered the trajectory
    controller, resumed from 18.9 mm, path complete); external cancel via the action (like UI Stop): arm motion zero
    from 0.5 s after the cancel, velocity controller left active, `Switch To Trajectory Controller` restored it, and a
    fresh run then completed.
  - scripts/tracking/run_objective.sh NAME [OUT.npz] [CANCEL_AFTER_S] records sun/truth/track/progress/joint states.
- 2026-11-11 step 2: real-system objective `Sinter Raster Continuous` (sinter_robot_hw/objectives/sinter_raster_continuous.xml).
  Generated from sinter_raster_closed_loop.xml so the proven preamble is identical: Switch To Trajectory Controller
  (top) → Scan_Initial → RealSense cloud → world → crop → FitPlaneRasterPath (RANSAC, min_points, tilt guard,
  focus_lift_m; line_subdivisions 2 so the reachability check gets ends + middles) → BeforeSinter → ephemeris seed
  (8 rolls) → Align Lens To Sun (tolerance or abort) → Raster Reachable With Roll (8 rolls, IK + collisions, fails
  with the arm still up) → planned collision-checked lead-in to raster point 0 with the chosen orientation →
  velocity_force_controller → TrackRasterWithSun (scan_speed_mps 0.001, gain_rot 0.5, gain_pos 1.0, 3 mm tracking
  abort, 1 deg pointing abort, pause on sun loss) → trajectory controller → BeforeSinter → Home → Scan_Initial.
  Failure branch: zero twist (behavior) → Switch To Trajectory Controller → planned retreat to BeforeSinter → FAIL.
  Load-checked on the mock (runs up to GetPointCloud, which the mock lacks). Not yet run on hardware.
  Hardware velocity_force_controller limits tightened (sinter_robot_hw ros2_controllers.yaml): joints 0.3 rad/s,
  1 rad/s²; Cartesian 0.02 m/s / 0.1 rad/s, 0.1 m/s² / 0.5 rad/s² (were 1.0, 2.0, 0.25/1.0, 2.0/4.0). Needs a
  hardware stack restart. record_trial.sh now also records /sinter/sun_error_stream, /sinter/track_error,
  /sinter/track_progress, /velocity_force_controller/command and the continuous objective's tunables.
  Not yet in it: step 3 (reachability at the ephemeris orientation for each point's time; the sun turns the lens
  ~0.25 deg/min during the scan), the O1 incidence check.
  Suggested first real run: sinter_robot_hw_testing (real arm + real RealSense + sim sun) with
  use_ephemeris_seed := false; then hardware step 0 (Controller Switch Test); then the real sun.
- 2026-11-11 centred square raster + singularity-aware roll (user request).
  - FitPlaneRasterPath `raster_size_m` (> 0): the raster is a square of that side centred on the fitted plane's
    centroid, lines along the plane's projection of world +X, independent of the crop extent (the crop only has to
    land on the bed and be a bit larger). Test CenteredSquareIgnoresCloudExtent.
  - Dexterity = inverse condition number of the focal-frame Jacobian with translation rows / 0.4 m (0 singular .. 1),
    src/dexterity.cpp (robot model from the BehaviorContext, RobotModelLoader fallback). Behaviors ComputeDexterity
    (with running min) and KeepBestRoll. Raster Reachable With Roll now outputs `min_dexterity` (worst check point).
  - Sinter Raster Continuous scores ALL 8 rolls (IK + collisions + worst-point dexterity) and takes the best, instead
    of the first that reaches; aborts before the lead-in if best < min_roll_dexterity (0.02). Crop 0.08 m,
    raster_size_m 0.04.
  - TrackRasterWithSun: `min_dexterity` (live abort, publishes /sinter/dexterity) and `gain_roll` (lens roll about
    its optical axis up the dexterity gradient via J^+ [0; axis], numerical ±0.02 rad; does not move focus or sun
    alignment; clamped by max_roll_rate_rps 0.01). Real objective: min_dexterity 0.01, gain_roll 0 (off by default).
  - Mock results: Test Roll Dexterity (BeforeSinter, test path): rolls 0/±45 reachable, worst-point dexterity
    0.122 / 0.066 / 0.084 → roll 0 chosen; the other 5 rolls fail IK. Tracking at 0.5 mm/s with gain_roll 0.05:
    dexterity 0.122 → 0.135 over the scan, true pointing median 0.021 deg / p95 0.045, path error max 0.04 mm.
    So typical poses here are ~0.07–0.13; thresholds 0.02 / 0.01 are conservative floors.
  - 19/19 unit tests. Real objective load-checked on mock (stops at GetPointCloud as expected).
- 2026-11-11 loop timing (TrackRasterWithSun logs a Timing summary at the end and publishes /sinter/track_loop_ms;
  mock stack on the i7-10875H laptop, 50 Hz target):
  - without dexterity: step mean 0.170 / p99 0.455 / max 2.37 ms; tick period mean 20.000 / p99 20.19 / max 21.3 ms.
  - with dexterity monitor + roll gradient: step mean 0.275 / p99 0.719 / max 4.72 ms (dexterity part mean 0.173 /
    p99 0.456 / max 4.19 ms); tick period mean 20.000 / p99 20.20 / max 22.1 ms. No overruns.
  - sun detection (separate callback) per 1280x720 frame: mean ~2.1 / p99 ~5 / max 11.5 ms.
  The dexterity math costs ~0.1 ms per step on average (my estimate of "tens of µs" was low by ~3-5x), ~0.5-1.4 % of
  the 20 ms budget; the loop holds its 20 ms period either way.
- 2026-11-11 pre-hardware review (independent code review + own pass). Fixed:
  - Camera watchdog (`camera_timeout_s` 3 s): no frames at all = camera/USB down → fail (a cloud still pauses).
  - DEFOCUS LIFT (`lift_m` 0.03, `lift_speed_mps` 0.01): the focus is held 30 mm off the bed along the lens axis
    whenever the sun is not locked (start, clouds) and is lifted before reporting ANY stop (success or failure), so the
    spot never dwells focused on one point and the controller switch + planned retreat happen defocused. Path only
    advances fully down. 30 mm at f/0.74 spreads the spot to ~4 cm (~100x less flux). /sinter/track_lift published.
    Exception: a BT halt (UI Stop / Timeout) stops immediately WITHOUT lifting.
  - Acquire bound (`max_acquire_error_rad` 0.15): a glint can no longer steer the lens without limit.
  - Finish timeout (`finish_timeout_s` 5 s): no hang at the last point if the arm settles 0.5–3 mm off.
  - Halt race closed with a publish mutex; images get their own mutually-exclusive callback group; dexterity check
    fails after 2 s if it cannot evaluate; timing vectors locked.
  - FitPlaneRasterPath: square must fit on the measured surface (within 5 mm) or the fit fails; lines centred in
    the square (raster_size 0.04, edge 0.005, spacing 0.004 → 8 lines, ±14 mm).
  - KeepBestRoll `min_improvement` 0.25: rolls tried nearest first, a farther roll must be 25 % better (avoids big
    wrist spins in the lead-in).
  - record_trial.sh: untracked files count as dirty; records /sinter/track_lift. .gitignore: scripts/tracking outputs.
  - Mock: sim camera `~/drop_camera` fault; objective `Test Camera Dropout` (expected FAIL + recovery).
  Mock results after fixes: all fault tests pass (sun loss: lift to 30 mm, zero path advance while lifted, resume;
  camera dropout: lift then fail then trajectory controller restored; stop/resume; UI cancel: arm still in 0.5 s).
  Known, not fixed: reachability/dexterity check uses independent IK per point (the controller follows the Jacobian
  from the lead-in configuration, so it could meet a branch/limit the check didn't see; the live dexterity abort +
  lift cover it); velocity_force_controller limits are global (Teleoperate also slower); via-Home skipped (joint-6
  cable wrap, same as the closed-loop objective); blob detector params duplicated with SunPoseFinder.
- 2026-11-14 FIRST HARDWARE RUN (sinter_robot_hw_testing: real JACO2 + simulated sun), `Track Raster With Sun Mock`
  with the user's settings (0.5 mm/s, gain_rot 0.5, gain_roll 0.075, lift 3 mm/s, min_dexterity 0.005).
  Earlier attempts the same day failed 3-5 s in with "Focal point 3.0-3.2 mm off the path" during the start lift at
  10 mm/s: velocity_force_controller uses the position interface, so the driver's P loop (kp 5/s) plus arm latency
  lag ~3 mm behind a 10 mm/s reference; the mock has no lag. Lift at 3 mm/s fixed it.
  Run (logged passively, scripts/tracking/hw_runs/hw_run1.npz): acquired 7.2 s, full 98 mm path, lift at end,
  controller restored. Scan path error median 0.29 / p95 0.67 / max 1.09 mm (lines p95 0.65, corners max 1.0) vs mock
  0.01/0.01/0.04; lift-phase error max 2.08 mm (limit 3); TRUE pointing median 0.047 / p95 0.112 / max 0.19 deg vs
  mock 0.021/0.045/0.077 (half the rot gain, sim jitter 0.5 px still on: sensor-vs-truth rms 0.05 deg/axis);
  dexterity 0.122 -> 0.135 (min 0.114, roll optimisation working on hardware); control step 0.18 / p99 0.34 / max
  2.2 ms; joint motion smooth (2.6 % of 0.2 s windows near-still, no stick-slip pattern).
