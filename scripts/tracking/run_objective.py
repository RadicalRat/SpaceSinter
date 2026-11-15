"""Run a MoveIt Pro objective by name via /do_objective and record tracking topics while it runs.
usage: run_objective.py NAME OUT.npz [CANCEL_AFTER_S]
CANCEL_AFTER_S cancels the goal after that many seconds, like pressing Stop in the UI."""
import sys, time, math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState

name, out = sys.argv[1], sys.argv[2]
cancel_after = float(sys.argv[3]) if len(sys.argv) > 3 else None
rclpy.init()
n = Node("objective_runner")
rec = {"sun": [], "truth": [], "track": [], "prog": [], "js": [], "dex": [], "lift": [], "log": []}
now = lambda: n.get_clock().now().nanoseconds * 1e-9
n.create_subscription(Vector3Stamped, "/sinter/sun_error_stream", lambda m: rec["sun"].append((now(), m.vector.x, m.vector.y, m.vector.z)), 50)
n.create_subscription(Vector3Stamped, "/sim_sun_camera/true_error", lambda m: rec["truth"].append((now(), m.vector.x, m.vector.y, m.vector.z)), 50)
n.create_subscription(Vector3Stamped, "/sinter/track_error", lambda m: rec["track"].append((now(), m.vector.x, m.vector.y, m.vector.z)), 50)
n.create_subscription(JointState, "/joint_states", lambda m: rec["js"].append(tuple([now()] + list(m.position[:6]))), 50)
n.create_subscription(Float64, "/sinter/track_lift", lambda m: rec["lift"].append((now(), m.data)), 50)
n.create_subscription(Float64, "/sinter/dexterity", lambda m: rec["dex"].append((now(), m.data)), 50)
n.create_subscription(Float64, "/sinter/track_progress", lambda m: rec["prog"].append((now(), m.data)), 50)
def onlog(m):
    if "objective" in m.name or "Track" in m.msg or "Switch" in m.msg or "Roll" in m.msg or "roll" in m.msg or m.level >= 30:
        rec["log"].append((now(), m.level, m.name, m.msg))
n.create_subscription(Log, "/rosout", onlog, 200)
ac = ActionClient(n, DoObjectiveSequence, "/do_objective")
if not ac.wait_for_server(timeout_sec=30):
    print("no /do_objective server"); sys.exit(2)
goal = DoObjectiveSequence.Goal(); goal.objective_name = name
t0 = time.time()
f = ac.send_goal_async(goal); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
gh = f.result()
if not gh or not gh.accepted:
    print("goal rejected"); sys.exit(3)
rf = gh.get_result_async()
cancelled = False
while not rf.done() and time.time() - t0 < 600:
    rclpy.spin_once(n, timeout_sec=0.1)
    if cancel_after is not None and not cancelled and time.time() - t0 > cancel_after:
        print(f"CANCELLING after {cancel_after:.0f} s"); cancelled = True
        cf = gh.cancel_goal_async()
res = rf.result().result if rf.done() else None
t_end = time.time()
while time.time() - t_end < (8.0 if cancel_after is not None else 0.0):  # keep watching the arm after a cancel
    rclpy.spin_once(n, timeout_sec=0.05)
if cancel_after is not None and rec["js"]:
    a = np.array(rec["js"]); tc = a[0, 0] + cancel_after
    v = np.degrees(np.abs(np.diff(a[:, 1:], axis=0)).max(1) / np.maximum(np.diff(a[:, 0]), 1e-3))
    for lo, hi in ((-3, 0), (0, 0.5), (0.5, 1), (1, 3), (3, 8)):
        m = (a[1:, 0] >= tc + lo) & (a[1:, 0] < tc + hi)
        if m.any():
            print(f"  joint speed {lo:+.1f}..{hi:+.1f} s around cancel: max {v[m].max():.4f} deg/s")
print("RESULT", name, "code", res.error_code.val if res else "timeout", "|", res.error_message if res else "", f"| {time.time()-t0:.1f} s")
for t, lvl, nm, msg in rec["log"][-25:]:
    print(f"  [{lvl}] {nm}: {msg[:200]}")
np.savez(out, **{k: np.array([r if k == "js" else r[:4] for r in v], float) if v else np.array([]) for k, v in rec.items() if k != "log"})
print({k: len(v) for k, v in rec.items()})
