# Trial analysis scripts

Python 3 venv with `mcap mcap-ros2-support numpy scipy matplotlib opencv-contrib-python-headless`. `$S` is a work dir.

1. `extract.py BAG.mcap $S/t2.pkl`: dumps the non-image topics from a sinter bag.
2. `fk.py $S/t2.pkl $S/t2_fk.npz`: FK focal-point / sun-camera poses from `/tf` + `/tf_static`. Focal +Z is the lens axis toward the sun.
3. `calib.py GOPRO_CALIB.MP4 $S/gopro_intrinsics.json`: ChArUco (11x8, DICT_6X6_250) intrinsics.
4. `detect_tags.py $S/tags_ch1.pkl 3 CH1.MP4` (repeat for `tags_ch2.pkl`): AprilTag 16h5 corners on every 3rd frame.
5. `FIX_DT=0.27 pose.py $S`: tag layout bundle adjustment, multi-tag PnP per frame, and a hand-eye fit with tag scale against FK. Writes `$S/t2_pose.npz`. Leave out FIX_DT to search the time offset (it is flat when the arm deflects).
6. `compare.py $S`: GoPro vs FK residuals, per-raster-line axis drift, and the plot.

The GoPro chapter frame counts and the timecode in `pose.py` are hard-coded for Trial2_10_30.
