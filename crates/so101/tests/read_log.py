# /// script
# requires-python = ">=3.12"
# dependencies = ["mcap>=1.6", "mcap-protobuf-support>=0.6"]
# ///
"""Contract 1: a log is readable from Python with no generated code. Decodes every message of
a log using only the schemas embedded in the file. Run by `python_reads_a_log_without_generated_code`
in log.rs, on the sample session.

usage: uv run crates/so101/tests/read_log.py LOG.mcap
"""
import hashlib
import sys
from collections import Counter

from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

with open(sys.argv[1], "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])
    header = {m.name: dict(m.metadata) for m in reader.iter_metadata()}
    attachments = {a.name: a.data for a in reader.iter_attachments()}
    counts = Counter()
    for schema, channel, message, pb in reader.iter_decoded_messages(log_time_order=True):
        counts[channel.topic] += 1
        if channel.topic == "/follower/state":
            assert len(pb.pose) == 6 and len(pb.counts) == 6, pb  # the pose and the raw counts behind it
            assert message.publish_time == pb.t_ns
        elif channel.topic.startswith("/cam/"):
            assert schema.name == "foxglove.CompressedImage"
            assert pb.format == "jpeg" and pb.data[:2] == b"\xff\xd8"
            t_capture = pb.timestamp.seconds * 1_000_000_000 + pb.timestamp.nanos
            assert message.publish_time == t_capture
            assert message.log_time > message.publish_time
        elif channel.topic == "/tick":
            assert len(pb.state) == 6 and len(pb.action) == 6 and len(pb.requested) == 6 and pb.wrist.sequence >= 0
            counts["clipped"] += len(pb.clipped)
        elif channel.topic == "/policy/chunk":
            assert pb.k * 6 == len(pb.actions)

session = header["so101.session"]
assert session["git_commit"], header
# The calibration file is in the log, and matches the hash in the header.
assert hashlib.sha256(attachments["calibration/follower.json"]).hexdigest() == session["calibration.follower.sha256"]
assert session["camera.wrist.device"].startswith("/dev/v4l/by-id/"), session
assert counts["clipped"] > 0, counts
assert header["so101.session_end"]["complete"] == "true", header
expected = {"/follower/state", "/leader/state", "/follower/actuators", "/follower/target", "/tick", "/cam/wrist", "/cam/scene", "/policy/chunk"}
assert expected <= set(counts), counts
del counts["clipped"]
print(f"python read {sum(counts.values())} messages on {len(counts)} topics without generated code")
