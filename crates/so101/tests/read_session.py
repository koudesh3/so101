# /// script
# requires-python = ">=3.12"
# dependencies = ["mcap>=1.6", "mcap-protobuf-support>=0.6"]
# ///
"""Contract 1: a log is readable from Python with no generated code. Decodes every message of a
log using only the schemas embedded in the file, and checks what contracts 1, 3, 6, 7 and 8 say a
log carries. Run on the sample session by `python_reads_a_session_without_generated_code`
(session.rs, with --sample, which also checks the sample's own content), and on real recordings.

usage: uv run crates/so101/tests/read_session.py SESSION.mcap [--sample]
"""
import csv
import hashlib
import sys
from collections import Counter
from pathlib import Path

from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

path = Path(sys.argv[1])
sample = "--sample" in sys.argv[2:]

with open(path, "rb") as f:
    reader = make_reader(f, decoder_factories=[DecoderFactory()])
    header = {m.name: dict(m.metadata) for m in reader.iter_metadata()}
    attachments = {a.name: a.data for a in reader.iter_attachments()}
    counts = Counter()
    wrist_frames, tick_frames = set(), []
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
            # Contract 3: received after captured (a stamp on arrival would make them equal).
            assert message.log_time > message.publish_time, (channel.topic, message.sequence)
            if channel.topic == "/cam/wrist":
                wrist_frames.add(message.sequence)
        elif channel.topic == "/tick":
            assert len(pb.state) == 6 and len(pb.action) == 6 and len(pb.requested) == 6 and pb.wrist.sequence >= 0
            counts["clipped"] += len(pb.clipped)
            tick_frames.append((message.sequence, pb.wrist.sequence if pb.HasField("wrist") else None,
                                pb.scene.sequence if pb.HasField("scene") else None))
        elif channel.topic == "/policy/chunk":
            assert pb.k * 6 == len(pb.actions)

session = header["so101.session"]
assert session["git_commit"], header
# Contract 6: every calibration in the log matches the hash in the header.
calibrations = [k[len("calibration."):-len(".sha256")] for k in session if k.startswith("calibration.") and k.endswith(".sha256")]
for name in calibrations:
    got = hashlib.sha256(attachments[f"calibration/{name}.json"]).hexdigest()
    assert got == session[f"calibration.{name}.sha256"], name
# Contract 7: each camera's device and read-back settings.
cameras = sorted({k.split(".")[1] for k in session if k.startswith("camera.") and k.endswith(".device")})
for role in cameras:
    assert session[f"camera.{role}.device"], role
    assert f"camera.{role}.settings" in session, role
assert header["so101.session_end"]["complete"] == "true", header
required = {"/follower/state", "/follower/target", "/tick"}
assert required <= set(counts), counts
# Contract 2: every frame a tick names is in the recording, from its first tick on.
named = [w for _, w, _ in tick_frames if w is not None]
if "/cam/wrist" in counts:
    missing = [(n, w) for n, w, _ in tick_frames if w is not None and w not in wrist_frames]
    assert not missing, f"ticks naming wrist frames not in the log (tick, frame): {missing[:5]}"

# Contract 1's exception: the scene camera beside the log, its frame times in a CSV.
scene = ""
if "scene" in cameras and not sample:
    prefix = str(path)[: -len(".mcap")]
    rows = list(csv.DictReader(open(f"{prefix}-scene.csv")))
    assert rows, "no scene frames"
    scene_frames = {int(r["sequence"]) for r in rows}
    missing = [(n, sc) for n, _, sc in tick_frames if sc is not None and sc not in scene_frames]
    assert not missing, f"ticks naming scene frames not in the CSV (tick, frame): {missing[:5]}"
    for a, b in zip(rows, rows[1:]):
        assert int(b["t_capture_ns"]) > int(a["t_capture_ns"]) and int(b["sequence"]) > int(a["sequence"])
        assert int(a["t_receive_ns"]) > int(a["t_capture_ns"])
    video = Path(f"{prefix}-scene.mkv")
    raw = Path(f"{prefix}-scene.mjpeg")
    assert video.exists() or raw.exists(), "scene video missing"
    scene = f"; scene beside it: {len(rows)} frames, {'encoded' if video.exists() else 'still raw'}"

if sample:
    assert calibrations == ["follower"], calibrations
    assert session["camera.wrist.device"].startswith("/dev/v4l/by-id/"), session
    assert counts["clipped"] > 0, counts
    expected = {"/follower/state", "/leader/state", "/follower/actuators", "/follower/target", "/tick", "/cam/wrist", "/cam/scene", "/policy/chunk"}
    assert expected <= set(counts), counts

counts.pop("clipped", None)
cams = ", ".join(f"{t} {counts[t]}" for t in sorted(counts) if t.startswith("/cam/"))
print(f"python read {sum(counts.values())} messages on {len(counts)} topics without generated code ({cams}){scene}; "
      f"{len(named)} of {len(tick_frames)} ticks name a wrist frame, all in the log")
