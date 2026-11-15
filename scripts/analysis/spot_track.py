"""Track the glowing sinter spot in a HERO8 eclipse-film clip by colour (orange, bright), per sampled frame.

    spot_track.py VIDEO OUT.npy STEP
Saves rows (t_video_s, x_px, y_px, area_px, peak_R). NaN when no orange pixels.
"""
import sys

import cv2
import numpy as np

video, out, step = sys.argv[1], sys.argv[2], int(sys.argv[3])
cap = cv2.VideoCapture(video)
fps = cap.get(cv2.CAP_PROP_FPS)
rows, k = [], 0
while cap.grab():
    if k % step == 0:
        _, img = cap.retrieve()
        b, g, r = [c.astype(np.int16) for c in cv2.split(img)]
        mask = ((r > 170) & (r - b > 70) & (r - g > 20)).astype(np.uint8)
        n, lab, st, cen = cv2.connectedComponentsWithStats(mask)
        if n > 1:
            j = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
            ys, xs = np.nonzero(lab == j)
            w = (r[ys, xs] - 160).astype(float)
            rows.append((k / fps, (xs * w).sum() / w.sum(), (ys * w).sum() / w.sum(), st[j, cv2.CC_STAT_AREA], r[ys, xs].max()))
        else:
            rows.append((k / fps, np.nan, np.nan, 0, 0))
    k += 1
np.save(out, np.array(rows, float))
print(k, "frames;", len(rows), "sampled;", int(np.isfinite(np.array(rows)[:, 1]).sum()), "with spot")
