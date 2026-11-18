# Trial analysis scripts

Python 3 venv with `mcap mcap-ros2-support numpy scipy matplotlib opencv-contrib-python-headless`. `$S` is a work dir.

1. `extract.py BAG.mcap $S/t2.pkl`: dumps the non-image topics from a sinter bag.
2. `fk.py $S/t2.pkl $S/t2_fk.npz`: FK focal-point / sun-camera poses from `/tf` + `/tf_static`. Focal +Z is the lens axis toward the sun.
3. `calib.py GOPRO_CALIB.MP4 $S/gopro_intrinsics.json`: ChArUco (11x8, DICT_6X6_250) intrinsics.
4. `detect_tags.py $S/tags_ch1.pkl 3 CH1.MP4` (repeat for `tags_ch2.pkl`): AprilTag 16h5 corners on every 3rd frame.
5. `FIX_DT=0.27 pose.py $S`: tag layout bundle adjustment, multi-tag PnP per frame, and a hand-eye fit with tag scale against FK. Writes `$S/t2_pose.npz`. Leave out FIX_DT to search the time offset (it is flat when the arm deflects).
6. `compare.py $S`: GoPro vs FK residuals, per-raster-line axis drift, and the plot.

`pose.py` defaults reproduce Trial2_10_30. Other trials: `TAG=0.030 FPS=60000/1001 FRAME0=<ROS time of frame 0> NAME=t14 INTRINSICS=.../hero12_4k60_intrinsics.json TAGS=$S/T14_tags.pkl DT_GRID=-10,10 pose.py $S ch1` (reads `$S/t14_fk.npz`, writes `$S/t14_pose.npz`). HERO12 clock on 2026-11-15 = MST + 1 h: frame0 = timecode - 1 h + 7 h (UTC).

7. `continuous_figs.py DATA OUT`: continuous-control (TrackRasterWithSun) trials: pointing / path error / dexterity vs time, ECDF vs the discrete trials.

### 2026-11-15 GoPro vs FK (continuous trials)

The GoPro clock was **~134-137 s fast** on 2026-11-15 (not just MST + 1 h), so the INDEX.txt / timecode frame0 values are 2+ minutes off and the old `DT_GRID=-10,10` search could not find the offset (its cost was flat because every candidate was wrong). Find the offset from speeds instead: `SYNC_RANGE=300 pose.py ...` (cross-correlates camera vs FK rotation and translation speeds; neither needs the hand-eye). Fitted offsets (ROS frame0 = timecode frame0 + dt): T14 -137.4, T16 -136.0, T19 -134.4, T20 -133.7 s. `gopro_diag.py` reports the same offsets from translation and rotation speeds separately (agree within 0.15 s); a 1.4 s error already drops the camera/FK rotation-angle ratio from 0.99 to 0.82.

8. `tag_poses.py OUT INTRINSICS.json 0.030 t14=T14_tags.pkl [...]`: tag layout (alternating bundle adjustment, much faster than pose.py's dense one) + per-frame multi-tag PnP with IPPE flip handling. Pass one clip per call for per-clip layouts (clips mostly seeing tags 4+7 still get a layout). Writes `OUT/<name>_campose.npz`.
9. `gopro_diag.py OUT/t14_campose.npz t14_fk.npz t14.pkl OUT/t14 --frame0 <timecode frame0>`: speed-correlation time sync (+-300 s), hand-eye-free checks (rotation-angle ratio, static jitter), hand-eye + scale fit `p_cam = Rx k (p_f + R_f y) + c` with standard errors, the travel ratio k per motion phase with block-bootstrap CIs, lens-axis residuals, scan tilt. `--t-max/--t-min` split a clip at a camera bump (T19 bumped at ~460 s video time). Writes `_diag.json/.npz/.png` and a pose.py-style `_pose.npz` for `gopro_vs_fk.py`.
