"""Detect AprilTag 16h5 corners in GoPro chapters; saves frame index (across chapters) -> {id: 4x2 corners}."""
import pickle
import sys

import cv2

out, step = sys.argv[1], int(sys.argv[2])
chapters = sys.argv[3:]
A = cv2.aruco
params = A.DetectorParameters()
params.cornerRefinementMethod = A.CORNER_REFINE_SUBPIX
params.cornerRefinementWinSize = 3
# 16h5 has few bits, so reject marginal decodes to keep false positives out.
params.maxErroneousBitsInBorderRate = 0.1
params.errorCorrectionRate = 0.0
det = A.ArucoDetector(A.getPredefinedDictionary(A.DICT_APRILTAG_16h5), params)

result = {}
k = 0
for path in chapters:
    cap = cv2.VideoCapture(path)
    while cap.grab():
        if k % step == 0:
            _, img = cap.retrieve()
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = det.detectMarkers(g)
            if ids is not None:
                result[k] = {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}
            if k % (step * 600) == 0:
                print(k, len(result), flush=True)
                pickle.dump(result, open(out, "wb"))
        k += 1
pickle.dump(dict(result=result, n_frames=k, step=step), open(out, "wb"))
print("done", k, "frames,", len(result), "with tags")
