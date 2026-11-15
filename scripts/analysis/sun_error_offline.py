"""Offline SunPoseFinder: blob-detect every recorded sun-camera frame -> continuous pointing error."""
import sys, glob
import numpy as np, cv2
from multiprocessing import Pool
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

bag, out = sys.argv[1], sys.argv[2]
p = cv2.SimpleBlobDetector_Params()
p.minThreshold, p.maxThreshold, p.thresholdStep, p.minRepeatability = 240, 255, 5, 1
p.filterByColor, p.blobColor = True, 255
p.filterByArea, p.minArea = True, 15
p.filterByCircularity, p.minCircularity = True, 0.5
p.filterByConvexity, p.minConvexity = True, 0.5
p.filterByInertia = False
DET = None

def work(item):
    global DET
    if DET is None:
        DET = cv2.SimpleBlobDetector_create(p)
    t, buf = item
    img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # node runs the detector on rgb8, same channel order here
    kps = sorted(DET.detect(rgb), key=lambda k: -k.size)
    if not kps:
        return (t, np.nan, np.nan, 0.0, 0)
    k = kps[0]
    return (t, k.pt[0], k.pt[1], k.size, len(kps))

def frames():
    with open(glob.glob(bag + "/*.mcap")[0], "rb") as f:
        r = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, ch, m, msg in r.iter_decoded_messages(topics=["/camera/image_raw/compressed"]):
            yield (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, bytes(msg.data))

with open(glob.glob(bag + "/*.mcap")[0], "rb") as f:
    r = make_reader(f, decoder_factories=[DecoderFactory()])
    for _, ch, m, info in r.iter_decoded_messages(topics=["/camera/camera_info"]):
        K = np.array(info.k).reshape(3, 3); W, H = info.width, info.height
        break
with Pool(12) as pool:
    res = np.array(pool.map(work, frames(), chunksize=32))
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
vx, vy = (res[:, 1] - cx) / fx, (res[:, 2] - cy) / fy
np.savez(out, t=res[:, 0], u=res[:, 1], v=res[:, 2], size=res[:, 3], nblobs=res[:, 4], vx=vx, vy=vy,
         err=np.arctan(np.hypot(vx, vy)), K=K, W=W, H=H)
print(len(res), "frames,", int(np.isfinite(vx).sum()), "with a blob; K", K[0, 0], K[0, 2], K[1, 2], W, H)
