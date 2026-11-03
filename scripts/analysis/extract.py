"""Dump the non-image topics of a sinter bag to a pickle for analysis."""
import pickle
import sys
from collections import defaultdict

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

bag, out = sys.argv[1], sys.argv[2]
SKIP_DECODE = {"/camera/image_raw/compressed", "/camera/camera_info"}
data = defaultdict(list)


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


with open(bag, "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])
    for _, channel, message in reader.iter_messages(topics=list(SKIP_DECODE)):
        data[channel.topic].append(message.log_time * 1e-9)
    keep = [c.topic for c in reader.get_summary().channels.values() if c.topic not in SKIP_DECODE]
    for schema, channel, message, msg in reader.iter_decoded_messages(topics=keep):
        t = message.log_time * 1e-9
        topic = channel.topic
        if topic == "/joint_states":
            data[topic].append((t, stamp(msg.header), list(msg.name), list(msg.position), list(msg.velocity)))
        elif topic in ("/tf", "/tf_static"):
            for tr in msg.transforms:
                q, p = tr.transform.rotation, tr.transform.translation
                data[topic].append((t, stamp(tr.header), tr.header.frame_id, tr.child_frame_id,
                                    (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        elif topic == "/joint_trajectory_admittance_controller/controller_state":
            data[topic].append((t, list(msg.joint_names), list(msg.reference.positions),
                                list(msg.feedback.positions), list(msg.error.positions)))
        elif topic in ("/sinter/sun_error", "/sinter/sun_ephemeris"):
            data[topic].append((t, stamp(msg.header), msg.header.frame_id, msg.vector.x, msg.vector.y, msg.vector.z))
        elif topic in ("/sinter/bed_plane",):
            p, q = msg.pose.position, msg.pose.orientation
            data[topic].append((t, msg.header.frame_id, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        elif topic == "/sinter/raster_path":
            data[topic].append((t, msg.header.frame_id, [((ps.pose.position.x, ps.pose.position.y, ps.pose.position.z),
                                                          (ps.pose.orientation.x, ps.pose.orientation.y,
                                                           ps.pose.orientation.z, ps.pose.orientation.w))
                                                         for ps in msg.poses]))
        elif schema.name == "std_msgs/msg/Float64":
            data[topic].append((t, msg.data))
        elif schema.name == "std_msgs/msg/String":
            data[topic].append((t, msg.data))
        elif topic == "/rosout":
            data[topic].append((t, msg.level, msg.name, msg.msg))
        else:
            data[topic].append((t, repr(msg)[:4000]))

with open(out, "wb") as f:
    pickle.dump(dict(data), f)
print({k: len(v) for k, v in data.items()})
