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
- 2026-11-14 hardware run 2 (sim jitter 0.5 -> 0.1 px, otherwise as run 1; hw_runs/hw_run2.npz): completed.
  Path error unchanged (0.27 / 0.68 / 1.13 mm; lift max 2.06 mm). TRUE pointing 0.040 / 0.096 / 0.178 deg (run 1:
  0.047 / 0.112 / 0.191), 2.8 % of scan over 0.115 deg (run 1: 4.4 %). Measured-vs-truth difference stayed
  ~0.04 deg/axis despite 5x less jitter, so it is timing (frame vs truth sample alignment while the real lens moves),
  not sensor noise. True error split: slow (>2 s) 0.025 deg (mock 0.016; gain_rot 0.5 vs 1.0, lag = 2x-sun rate /
  gain) and fast (<2 s) 0.022 deg/axis (mock 0.010: real-arm response). Both show a 0.18 Hz component, likely the loop
  with the ~4 Hz sim camera (Python renderer); the real camera is 28 Hz.
- 2026-11-14 hardware run 3 (gain_rot 1.0, else as run 2; hw_runs/hw_run3.npz): completed, but pointing got WORSE:
  TRUE 0.098 / 0.189 / 0.252 deg, 37 % of scan over 0.115 deg (run 2 at gain 0.5: 0.040 / 0.096 / 0.178, 2.8 %);
  path error also up (0.45 / 0.81 / 1.39 mm). The true error is dominated by a ~0.2 Hz oscillation (peak-to-peak
  0.46 deg at gain 1.0 vs 0.33 at 0.5; mock 0.10). The sim camera runs at only ~4 Hz and its measurement lags truth
  by 0.25 s; with the real arm's response lag on top, the loop is delay-limited and higher gain just rings.
  Conclusion: keep gain_rot 0.5 in hw_testing; the real Arducam (28 Hz, less latency) should allow more gain, so
  retune there. Making the sim camera fast (render a crop or lower res) would make hw_testing representative.
- 2026-11-14 hardware run 4 (sim camera image_scale 0.25, sun 1.5 deg): blob never detected (2.3 px radius < detector
  limits), 0 % of 1369 frames; tracker stayed lifted 30 mm with the lens still and the path at 0 until the user
  stopped it — the no-sun pause/lift behaving as designed on the real arm.
- 2026-11-14 hardware run 5 (image_scale 0.25 + sun 4.0 deg → 30 Hz, measurement lag 0.03 s; real-speed drift;
  jitter 0.1 px ≈ 0.033 deg/axis; gain_rot 0.33; hw_runs/hw_run5.npz): BEST so far. TRUE pointing 0.036 / 0.091 /
  0.148 deg, 1.2 % of scan over 0.115 deg; path 0.28 / 0.64 / 1.06 mm; lift max 1.84 mm; acquire 10.4 s (low gain).
  A ~0.18 Hz component remains (0.26 deg p-p, ~half the error power on hardware). It is also in the mock with roll
  optimisation OFF (26 % of power), so it is neither the camera rate nor the roll optimisation; source not yet found.
- 2026-11-14 hardware run 6 (as run 5 but gain_roll 0; hw_runs/hw_run6.npz): BEST. TRUE pointing 0.032 / 0.070 /
  0.121 deg, 0.0 % over 0.115 deg; path 0.28 / 0.54 / 0.90 mm; lift max 1.34 mm. Roll optimisation (run 5) cost
  accuracy (0.036/0.091/0.148, 1.2 %) and turned joints 4 and 6 ~19 deg each over the scan (vs 2-4 deg without),
  while the MINIMUM dexterity was the same (0.111 vs 0.112); it only raised the end value (0.136 vs 0.123). The
  0.15-0.25 Hz share of error power dropped 49 % -> 30 % (mock without roll: 26 %), so the roll motion amplified
  the wobble on hardware but is not its root. Recommendation: keep gain_roll 0 (default in the real objective).
- 2026-11-14 hardware run 7 (as run 6 but gain_rot 0.5; hw_runs/hw_run7.npz): completed. Acquire 8.1 s (run 6: 10.9);
  TRUE pointing 0.035 / 0.071 / 0.124 deg, 0.1 % over 0.115 (run 6: 0.032 / 0.070 / 0.121, 0 %); path 0.26 / 0.51 /
  0.87 mm; lift-phase path error max 2.26 mm (run 6: 1.34; limit 3 mm). 0.18 Hz share 45 % (run 6: 30 %), same p-p.
  So 0.5 ≈ 0.33 in accuracy with faster acquisition; the lift error is the margin to watch (along/across-lift check
  still recommended before the real sun).
- 2026-11-14 hardware run 8 (as run 7 + gain_roll 0.025; hw_runs/hw_run8.npz): completed. Wrist still turned ~18-19 deg
  (j4 17.9, j6 19.0) — the same as at 0.075 — so the gain only sets how fast the roll climbs to the same local
  dexterity maximum, not how far. Dexterity end 0.135 vs 0.124 without roll, but the scan MINIMUM is unchanged
  (0.122 vs 0.121; the minimum is at the start, before the roll moves). Cost: pointing p95 0.094 vs 0.071 deg (1.8 %
  over tol vs 0.1 %), path p95/max 0.83/1.41 vs 0.51/0.87 mm. Conclusion: on this path roll optimisation buys no
  safety margin and costs accuracy at any gain; keep gain_roll 0. Up-front roll selection (KeepBestRoll) is the
  useful part.
- 2026-11-14 figure hw_runs/runs_pointing_and_dexterity.png (pointing + dexterity vs scan time, runs 1-8 except 4).
  Correction to the run 6/8 notes: with roll optimisation the scan MINIMUM dexterity is ~3-5 % higher (0.122-0.126 vs
  0.120-0.121), because the roll already starts moving during acquisition; by the end it is ~10 % higher. Still a
  small margin gain for the accuracy cost.
- 2026-11-14 CHECK of the "true pointing error" method (user asked):
  - Definition (sim_sun_camera.py): each rendered frame, the sun's world direction is rotated into the camera optical
    frame using the latest world→ee_camera_optical_link TF (FK from /joint_states); truth = (dx/dz, dy/dz) tan offsets
    and acos(dz). Same TF and sun direction that draw that frame; jitter is added only to the drawn blob, not to truth.
  - Camera optical axis vs lens (focal) axis in the URDF: 0.0003 deg apart, so camera-axis error = lens-axis error.
  - acos(dz) vs atan(hypot(tan x, tan y)): agree to 1e-9 deg.
  - Measured (sensor) minus truth, matched to the SAME frame (truth published just before the measurement): bias
    < 0.003 deg; rms 0.010 deg/axis at 1280x720 (expected jitter 0.008) and 0.040 at 320x180 (expected 0.033; rest is
    centroid quantisation of the small blob). CORRECTION: the earlier note (run 2) that the ~0.04 deg measured-vs-
    truth difference was "timing, not noise" was an alignment artefact of my nearest-sample matching; with
    same-frame matching it is just the jitter.
  - Caveat: "true" here = pointing of the arm's KINEMATIC MODEL (FK) at the sim sun, because the sim sun is placed
    from the FK pose. It cannot see arm deflection, joint-calibration or camera-mount errors — on the real sun those
    appear (the Trial 2 GoPro showed ~0.6 deg/100 mm that FK misses). The sun sensor closes the loop on them; this
    sim truth does not include them.
  - The figure's 2 s average of |error| reads slightly high (averaging magnitudes); the medians/percentiles in the
    tables use per-frame truth.
- 2026-11-14 SUN FEEDFORWARD added to TrackRasterWithSun (`sun_feedforward`, `latitude_deg`, `north_heading_deg`,
  `sun_feedforward_scale`): w_ff = -(2*pi/86400) * scale * celestial-pole axis (north at heading CCW from +X, tilted
  up by latitude), added to the P command before the rate clamp; also applied while the sun is hidden and during the
  final lift. Test SunFeedforward.MatchesTheEphemerisMotion: rotating the ephemeris sun by w_ff for 60 s lands within
  2 % of the ephemeris 60 s later (3 dates/times); a 10 deg heading error leaves < 15 % of the rate to feedback.
  Enabled in Sinter Raster Continuous ({latitude_deg}, {north_heading_deg}) and the mock objective (scale 1.0 = sim
  drift at real speed; set the scale to the sim's drift ratio if the sim drift is changed).
- 2026-11-14 review of Sinter Raster Continuous + tracker. Fixed: the real objective still lifted at 10 mm/s (the
  rate that failed on hardware) → 3 mm/s; during lifts the tracking abort now checks the error ACROSS the lift
  against max_tracking_error_m and ALONG it against 3x (the along-lift error is driver/arm lag that only moves the
  focus up/down); the image callback group is reused instead of created per run. 20/20 tests (scratch build; live
  install not yet rebuilt). Not changed (usage notes): use_ephemeris_seed must be false with the simulated sun;
  crop box position is hard-coded; via-Home still skipped; a UI Stop does not lift; reachability uses per-point IK.
- 2026-11-15 REVIEW FIXES implemented (findings 1-6 of the 2026-11-14 review + 10 follow-up findings from an
  AGENTS.md-persona reviewer subagent) and tested on the mock:
  - Tracker: hold band (scan_tolerance_rad 0.004, debounced hold_lift_s 0.1: lift + re-acquire instead of scanning
    off-sun; the scan no longer gates on each frame's error); start_lifted (lead-in goes to the LIFTED point 0);
    lift_m 0.08 in the objectives; settle gate after every descent (resume when < resume_tolerance_m 0.5 mm across
    the lens axis and < 3 mm along; after 2 s FAIL if > 1 mm across or > 3 mm along, else warn and resume); the
    looser across/along tracking check also applies while settling; lift axis re-read every tick; the lifted
    reference turns with the lens, so v_ff += lift * (w x axis) (also while finishing); sun feedforward projected
    perpendicular to the lens axis; lift_speed_mps default 0.003; sun_feedforward read strictly (fails if
    unreadable) and logged at start.
  - New SimulateRasterScan (kinematic rehearsal from ONE seed: descend -> raster -> lift, DLS Newton = exact IK on
    the nearest branch, orientation turning with the projected feedforward; fails on position residual > 0.5 mm,
    orientation residual > 1 mrad, joint limit, joint speed > 0.3 rad/s; path must be in the model frame) +
    OffsetPosesAlongAxis; roll selection now scores each roll by its simulated scan, collision-checks the simulated
    trajectory (ValidateTrajectory, last sample always included), and KeepBestRoll carries the seed
    (require_joint_state). Lead-in = Move to Joint State to that seed. Tracker and simulator share
    {sun_feedforward}/{sun_feedforward_scale}. Objective warns if gain_roll > 0 (the scored scan holds roll).
  - scripts/tracking/gen_continuous_objectives.py regenerates that section of Sinter Raster Continuous and derives
    Sinter Raster Continuous Mock (synthetic 3-line path); now idempotent (md5 unchanged on a 2nd run).
  - Tests: 20/20 gtests (scratch build). Live install rebuilt; mock objective symlinked into install.
  - Mock results (scripts/tracking/mock_runs/review_fixes_*.npz):
    - Sinter Raster Continuous Mock: SUCCESS. Rolls 0 and -45 passed (others IK-fail at the mock pose); roll 0 kept
      (sim min dexterity 0.093, peak joint speed 0.019 rad/s, wrist travel 16.9 deg). Tracker started 80.0 mm lifted
      at the seed, descended in 26.7 s, scanned 98 mm at 2 mm/s with no holds: path error mean/p95/max
      0.039/0.117/0.247 mm, true pointing 0.006/0.011/0.015 deg, MEASURED min dexterity 0.092 (sim predicted
      0.093), final 80 mm lift 26.7 s.
    - Hold band (Track Raster With Sun Mock + sim sun stepped +0.5 deg in elevation at 15 % progress): lift started
      0.14 s after the true error crossed the 0.229 deg band, peak lift 6.4 mm, re-acquired ~2 s, path frozen at
      14.78 mm throughout, resumed, completed. Focused (lift < 3 mm) above the band for 1.1 s total = debounce +
      time to climb 3 mm at 3 mm/s.
    - Test Sun Loss Pause, Test Stop And Resume: SUCCESS; path frozen during every lift. Test Camera Dropout:
      fails by design after the 3 s watchdog, lift completed to 30 mm first.
  - NOT exercised by the mock: the settle gate opened within one tick after every descent (the mock arm has no
    lag), so the 2 s timeout / fail path and the slow-descent warning are untested until hardware. Deferred
    (reviewer finding): up to ~1-1.5 s focused dwell at the resume point while settling on hardware (option:
    advance at ~0.3 V while settling). Reviewer's "add return after the camera-watchdog finish()" was REJECTED:
    it would skip the finishing (lift) branch on every later tick.
- 2026-11-15 SECOND REVIEW (Fable reviewer, AGENTS.md persona, read-only). No high-severity findings. Verified each
  against the code; fixed:
  - M1 a frame with no blob no longer replaces the last valid sun measurement (one bad frame used to pause + lift +
    re-settle); loss is declared once the last valid frame is older than sun_timeout_s (0.5 s).
  - M2 focal-frame TF staleness: older than 0.5 s (or lookup failing) → fail and lift OPEN LOOP along the last lens
    axis (the controller still moves by its own joint states); before: a frozen pose kept the loop running on a
    constant error, and a TF exception stopped focused with no lift.
  - M3 callback lifetime: subscription/timer lambdas hold a shared CallbackGuard (shared_mutex + alive flag);
    teardown (and the new destructor) waits for callbacks in flight and marks it dead. stop() no longer touches
    timer_ from the timer thread (raced with teardown resetting it).
  - L2 return after the camera-watchdog finish when lift_m = 0 (stop() already ran). L3 hold-band debounce also
    needs 2 distinct camera frames over the band (a slow real camera re-reads one frame for many ticks).
    L4 start tolerance capped at max_tracking_error_m. L5 at the path end while lifted: finish (success) without
    descending again. L6 the simulator skips zero-length segments and the output trajectory times strictly increase.
    L7 generator parses both XMLs before writing either, only ADDS missing Parameters (keeps hand tuning), and
    asserts the tracker and SimulateRasterScan read the same blackboard keys.
  - Rejected nothing this round; earlier "return after watchdog finish" is now done the safe way (only when done_).
  - Open (user decisions): M4 the controlStep state machine has no unit tests (needs extracting a pure step
    function; also the only way to test the settle timeout off hardware); L1 max_pointing_error_rad can never fire
    because the 0.1 s hold band pre-empts it (remove, or apply while re-acquiring); L6 the simulator rehearses the
    lift only at the first/last point, not a mid-raster lift on the scan's branch.
  - 21/21 gtests. Mock retest (scripts/tracking/mock_runs/review2_*.npz): Sinter Raster Continuous Mock SUCCESS
    (path err mean/p95/max 0.039/0.116/0.247 mm, true pointing 0.004/0.008/0.016 deg, dexterity 0.092);
    hold band (0.5 deg sun step) lifted 7.8 mm, resumed at 14.8 mm, completed; Sun Loss Pause and Stop And Resume
    SUCCESS; Camera Dropout fails by design after lifting. No false TF-stale trips. TF-stale path itself untested.
- 2026-11-15 HARDWARE, REAL SUN (sinter_robot_hw; Arducam 28 Hz). Two attempts failed the start check at 4.1/4.5 mm:
  the JTAC accepts a move within default_goal_tolerance 0.05 rad and the JACO2 settles with steady-state joint error;
  my 3 mm start-tolerance cap (L4) made that fatal. Fix: start convergence (converged_): up to start_tolerance_m
  (objective 0.01) is closed while lifted before acquiring; fail if > tol+2 mm or > 15 s.
  TRIAL13 (Elements sinter_bags/2026-11-15_124706_Trial13; gain_rot 0.25, focus_lift -0.06, heading 121): never left
  0.0 mm. 14 hold-band trips in 45 s, all during the 80 mm DESCENT (spot defocused, lift never < 15 mm); ended on
  "3.11 mm across the lifted reference" -> lift + retreat. Analysis: not the FF (7.3e-5 rad/s, 10-30x below the
  feedback); camera noise 0.013 deg/frame, 0.2 % frames without blob. FK from measured joints reproduces the measured
  pointing swings (e.g. -0.05 -> -0.32 -> +0.10 deg FK vs -0.09 -> -0.38 -> +0.08 measured in 0.5 s, corr 0.64), so
  real joint motion: each hold reversed the lift (velocity step -3 -> +3 mm/s + feedback, |v_cmd| up to 7 mm/s), the
  driver's per-joint P loop (kp 5/s, lag = qdot/kp) turned that into 0.3-0.5 deg lens tilt and 3-4 mm focal shift,
  which re-tripped the 0.229 deg band: a limit cycle. Steady 3 mm/s descent alone: p95 0.22 deg, track err < 1 mm.
  Fixes (live install, mock not rerun): out-of-band while defocused (lift > 3 mm) now HOLDS the height (ramped stop)
  until back within acquire_tolerance instead of re-lifting; full lift/re-acquire only when focused. Lift velocity
  ramped (kLiftAccel 3 mm/s^2, stopping profile at 0.8x; replica sim: no overshoot, residual snap < 0.15 mm/s).
  scan_tolerance 0.004 -> 0.005 rad (0.29 deg, within R2 ~0.3); gain_rot back to 0.5. Option not taken yet: driver
  kp 5 -> 10 to halve all lags.
- 2026-11-15 TRIAL14 (gain_rot 0.5, scan 0.75 mm/s, scan band 0.005, ramped lift + descent hold): FULL RASTER
  COMPLETED on the real sun (420 mm, 39 points, ~10 min), 80 mm lift at the end; then cancelled by the user during
  the retreat moves. TRIAL15 (same settings) failed 1.1 s in: "3.01 mm across the lifted reference". Bag: the
  lead-in Move to Joint State returned while joints still moved 4-8 deg/s (JTAC goal tolerance 0.05 rad, driver lag),
  so the velocity controller stopped the arm ~5 mm short; then a 0.40-0.45 deg post-lead-in sun error (kinematic
  error FK misses) was corrected at gain 0.5 while lifted, the wrist joints' lag moved the focus 3 mm sideways.
  Fixes: WaitForDuration 1.5 s after the lead-in (generator); across-axis limit 3x while the lift is above the depth
  of focus (strict limit near focus; the settle gate re-centres before scanning).
- 2026-11-15 VERIFIED REAL-SUN TRIALS (Elements sinter_bags; full raster 420 mm, 39 points, 40 mm square, 2.5 mm
  spacing, 0.75 mm/s, gain_rot 0.5, band 0.005 rad, focus_lift -0.06, heading 121; all "Path complete" + 80 mm lift,
  objective then cancelled by the user during the retreat moves):
  | trial | start | scan s | path err mean/p95/max mm | pointing p50/p95/max deg | min dex | sun el | incidence | DNI |
  | 14 | 13:16 | 560.0 | 0.51/1.09/1.76 | 0.046/0.143/0.250 | 0.076 | 27.8-27.0 | 30.6-31.4 | 908-930 |
  | 16 | 13:40 | 563.4 | 0.47/1.02/2.14 | 0.041/0.112/(1 glare frame) | 0.092 | 25.7-24.7 | 34.7-36.1 | 882-899 |
  | 19 | 14:10 | 560.0 | 0.40/0.88/1.61 | 0.040/0.125/0.218 | 0.099 | 22.5-21.3 | 30.3-31.5 | 825-835 |
  | 20 | 14:26 | 560.0 | 0.51/1.01/1.60 | 0.040/0.097/0.180 | 0.074 | 20.6-19.4 | 43.2-45.0 | 830 (1 sample) |
  Speed 0.750 mm/s (0.745 in 16 incl. pauses); 0 frames over the 0.286 deg band; no pointing holds; progress
  monotonic; camera 28.1 Hz (max gap 0.06 s), joint_states 100 Hz, twist 50 Hz (max gap 40 ms), loop p99 < 1 ms.
  Planes: 14 tilt 31.6 RMS 1.9 (767 out), 16 30.8/1.8 (491), 19 37.3/0.7 (437), 20 32.2/2.2 (803); raster ~3 cm
  uphill of the crop centre (focus_lift along the normal). Trial16: two 0.7/1.0 s camera dropouts (spot vanishes,
  background unchanged: beam blocked by a shadow, DNI steady) at 87.7 and 278.0 mm -> "Sun lost"; with the ramped
  lift the focus only rose 0.24/0.94 mm (inside the 3 mm depth of focus), so those two spots got ~1.1/2.2 s extra
  near-focused dwell; one glare frame read 41 deg right after. Path/pointing errors are FK-based / sun-camera
  measured, not true spot placement (kinematic error FK misses ~0.6 deg/100 mm). Trial20 incidence 43-45 deg (cos 0.72).
  Discrete Trial 11 for comparison: pointing median 0.093 / p95 0.19 deg; continuous ~2x better. Caveat: all four
  are git bb794e2-dirty; Trial14 ran before the 13:35 changes (settle wait, 3x lifted across limit), 16-20 after
  (if the stack was restarted). Commit the current tree to pin 16-20.
