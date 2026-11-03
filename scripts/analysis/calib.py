"""ChArUco intrinsic calibration of the GoPro from the board-waving video."""
import json
import sys

import cv2
import numpy as np

video, out = sys.argv[1], sys.argv[2]
A = cv2.aruco
dictionary = A.getPredefinedDictionary(A.DICT_6X6_250)
STEP = 4

cap = cv2.VideoCapture(video)
frames = []
k = 0
while True:
    ok = cap.grab()
    if not ok:
        break
    if k % STEP == 0:
        ok, img = cap.retrieve()
        frames.append((k, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)))
    k += 1
size = frames[0][1].shape[::-1]
print(len(frames), "frames sampled of", k, "size", size)


def make_board(ratio, legacy):
    b = A.CharucoBoard((11, 8), 1.0, ratio, dictionary)
    b.setLegacyPattern(legacy)
    return b


def detect(board):
    det = A.CharucoDetector(board)
    res = []
    for k, g in frames:
        cc, cid, mc, mid = det.detectBoard(g)
        if cid is not None and len(cid) >= 12:
            obj, img = board.matchImagePoints(cc, cid)
            res.append((k, obj, img, cid))
    return res


# Pick board layout (legacy pattern flag, marker/square ratio) by detection count and RMS on a subset.
best = None
for legacy in (False,):
    for ratio in (0.75,):
        res = detect(make_board(ratio, legacy))
        if len(res) < 10:
            print(f"legacy={legacy} ratio={ratio}: {len(res)} views")
            continue
        sub = res[:: max(1, len(res) // 25)]
        rms, *_ = cv2.calibrateCamera([r[1] for r in sub], [r[2] for r in sub], size, None, None,
                                      flags=cv2.CALIB_RATIONAL_MODEL)
        n = sum(len(r[3]) for r in res)
        print(f"legacy={legacy} ratio={ratio}: {len(res)} views, {n} corners, subset rms {rms:.3f}")
        if best is None or (n, -rms) > (best[0], -best[1]):
            best = (n, rms, legacy, ratio, res)

_, _, legacy, ratio, res = best
# Spread views over the video, cap the count so calibration stays fast.
views = res[:: max(1, len(res) // 80)]
obj = [r[1] for r in views]
img = [r[2] for r in views]

results = {}
for name, flags in (("pinhole_k3", 0), ("rational", cv2.CALIB_RATIONAL_MODEL)):
    rms, K, D, rv, tv, _, _, per_view = cv2.calibrateCameraExtended(obj, img, size, None, None, flags=flags)
    results[name] = dict(rms=rms, K=K.tolist(), D=D.ravel().tolist(), per_view_max=float(per_view.max()))
    print(name, "rms", round(rms, 3), "K", K.round(1).tolist(), "D", D.ravel().round(4).tolist())

# Coverage of the image by detected corners (calibration is only trustworthy where there were corners).
allpts = np.concatenate([i.reshape(-1, 2) for i in img])
H, _, _ = np.histogram2d(allpts[:, 0], allpts[:, 1], bins=[8, 6], range=[[0, size[0]], [0, size[1]]])
print("corner coverage grid (8x6 cells, count>0):\n", (H.T > 0).astype(int))

json.dump(dict(video=video, image_size=size, board=dict(squares=[11, 8], dictionary="DICT_6X6_250",
                                                          marker_to_square=ratio, legacy=legacy),
               views=len(views), models=results), open(out, "w"), indent=1)
