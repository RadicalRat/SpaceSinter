"""Passive logger: records tracking topics during the NEXT objective run, never sends goals.
Starts recording at the objective's 'starting at time' log, stops a few s after it ends (success, failure, stop) or at
the timeout. usage: log_run.py OUT.npz [TIMEOUT_S]"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log

out = sys.argv[1]; timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 1500
rclpy.init(); n = Node("passive_track_logger")
now = lambda: n.get_clock().now().nanoseconds * 1e-9
rec = {k: [] for k in ("sun", "truth", "track", "prog", "lift", "dex", "loop", "js")}
logs = []; state = {"started": None, "ended": None, "name": ""}
def add(k, row):
    if state["started"] is not None and state["ended"] is None or (state["ended"] and now() - state["ended"] < 3):
        rec[k].append(row)
v3 = lambda k: (lambda m: add(k, (now(), m.vector.x, m.vector.y, m.vector.z)))
f64 = lambda k: (lambda m: add(k, (now(), m.data)))
n.create_subscription(Vector3Stamped, "/sinter/sun_error_stream", v3("sun"), 50)
n.create_subscription(Vector3Stamped, "/sim_sun_camera/true_error", v3("truth"), 50)
n.create_subscription(Vector3Stamped, "/sinter/track_error", v3("track"), 100)
n.create_subscription(Float64, "/sinter/track_progress", f64("prog"), 100)
n.create_subscription(Float64, "/sinter/track_lift", f64("lift"), 100)
n.create_subscription(Float64, "/sinter/dexterity", f64("dex"), 100)
n.create_subscription(Float64, "/sinter/track_loop_ms", f64("loop"), 100)
n.create_subscription(JointState, "/joint_states", lambda m: add("js", tuple([now()] + list(m.position[:6]))), 100)
def onlog(m):
    t = now()
    if "starting at time" in m.msg and state["started"] is None:
        state["started"] = t; state["name"] = m.msg.split("`")[1] if "`" in m.msg else m.msg
        print(f"[{time.strftime('%H:%M:%S')}] recording: {m.msg[:120]}", flush=True)
    if state["started"] is not None and ("TrackRasterWithSun" in m.msg or "Execution of Behavior Tree" in m.msg or "LogMessage" in m.msg):
        logs.append((t - state["started"], m.level, m.msg)); print(f"  +{t - state['started']:6.1f}s [{m.level}] {m.msg[:200]}", flush=True)
    if state["started"] is not None and state["ended"] is None and (
            "did not succeed" in m.msg or "was cancelled" in m.msg or "Path complete." in m.msg and "Lifting" not in m.msg
            or "trajectory controller restored" in m.msg):
        state["ended"] = t
n.create_subscription(Log, "/rosout", onlog, 500)
print(f"[{time.strftime('%H:%M:%S')}] listening; start the objective when ready", flush=True)
t0 = time.time()
while time.time() - t0 < timeout:
    rclpy.spin_once(n, timeout_sec=0.1)
    if state["ended"] and now() - state["ended"] > 4:
        break
np.savez(out, **{k: np.array(v, float) for k, v in rec.items()}, started=state["started"] or 0, name=state["name"])
open(out.replace(".npz", ".log.txt"), "w").write("\n".join(f"+{a:.1f}s [{b}] {c}" for a, b, c in logs))
print("saved", out, {k: len(v) for k, v in rec.items()}, flush=True)
