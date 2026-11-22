# Phase 0 design: contracts and log

Status: built, 2026-11-22. Implements Phase 0 of [part2-spec.md](part2-spec.md).

Built as designed, in `crates/contracts` and `crates/session-log`, with three differences: the log
crate's package is named `session-log` (`log` clashes with the widely used crate of that name);
the reader loads a whole file into memory (streaming reads can come when logs get long); and
recovery keeps whole messages from a truncated chunk too, so a cut file loses less than its last
chunk.

**Exit criteria (from the spec):** a log written and read back gives identical records, and the
replay clock reproduces a run's timing. No hardware is needed, and today's teleop is untouched; it
moves onto the bus and traits in Phase 1.

This doc starts with the promises each side can rely on, then what happens when something goes
wrong. Those are the decisions that are expensive to change. The crates, dependencies and topic
names come last, as the implementation of those promises.

## Contracts

### 1. Logs stay readable forever

- A session log is one MCAP file. MCAP is the envelope (records, chunks, index); each message
  inside is protobuf; a camera frame's picture is the camera's own JPEG bytes inside a protobuf
  message.
- Every file embeds the exact schema each message was written with, so Python and Foxglove decode
  any log, old or new, without generated code.
- The `.proto` files in `proto` (package `so101.v1`) are the single definition of every logged
  message. Evolution rules: never reuse or renumber a field; removed fields become `reserved`;
  breaking changes go in `so101.v2`. A test compares the compiled schemas against a frozen snapshot
  in the repo, so a breaking change fails `just check`.
- A log is immutable once written. Its header records the git commit, whether the tree was dirty,
  the host, the clock source, a monotonic + wall-clock pair read together at session start, and the
  config.
- Only messages something produces get defined. Adding a message later is a non-breaking change;
  a premature one would be frozen by the snapshot.

### 2. Tick is the truth about what the loop saw

The central promise. At every control tick (30 Hz) the loop logs one `Tick`: the state it used,
the frame it used from each camera (by identity), the action it sent, and the source of that
action. **Anything downstream (the LeRobot converter, analysis, probes) reads the loop's view of
the world only from `Tick` and the messages it references. Nothing re-derives it from timestamps.**
So a converted episode matches what the policy or operator actually acted on, by construction.

### 3. What the timestamps and sequence numbers mean

- All times are u64 nanoseconds of the host's monotonic clock (`CLOCK_MONOTONIC`). It never jumps,
  unlike wall time under NTP. Times compare only within one log; real dates come from the header's
  monotonic/wall pair.
- Every MCAP record carries two times. **`publish_time`** is when the thing happened: a frame's
  capture, a servo read, a decision. **`log_time`** is when our process received it. The event time
  is also inside the message, so a message read alone is complete. Channels line up by
  `publish_time`, and staleness is `log_time - publish_time`, both from the record headers alone.
- What "capture time" means depends on the frame source, and the header records which: for a V4L2
  camera on Linux it's the kernel's buffer timestamp (the same clock as `CLOCK_MONOTONIC`); for a
  synthetic camera it's when the frame was generated; for a replayed camera it's the original
  capture time, offset onto the replay's clock.
- Every record's **`sequence`** is a per-source counter set by the producer: the kernel's frame
  number for a camera, the producer's own count for joint state, targets and ticks.
- **A sequence gap proves the host lost something; it doesn't prove the camera didn't skip a
  frame.** The V4L2 counter counts frames the driver received, not frames the camera never sent.
  That was camera 2's failure in the camera spike: under auto exposure it fell to 16-20 fps with no
  gaps in its counter. So a drop is defined as **a sequence gap, or a capture interval longer than
  1.5 × the camera's nominal period**. The reader and the converter report both kinds, separately.

### 4. What a policy guarantees

- A `Policy` takes and returns **degrees** (gripper 0-100), exactly the units of `JointState` and
  `JointTarget`. Nothing outside the policy ever sees normalized values.
- A policy **owns its normalization.** The spec's export puts it inside the ONNX graph; if it ever
  lives in a separate stats file instead, that file (JSON) names the model it belongs to.
- A policy **refuses to load** if its normalization doesn't match its model, so a model with the
  wrong stats (which moves the arm plausibly and wrongly) can't run.
- Every `Chunk` carries the checkpoint ID that produced it.

### 5. The servo loop keeps to its grid

- Slot *n* starts at `t0 + n × period`, a fixed grid (90 or 120 Hz, see below), never one period
  after the previous work finished, so the loop can't drift.
- **An overrun skips, never starts late.** If slot *n*'s work is still running when slot *n+1*
  should start, slot *n+1* is skipped, and the loop sleeps until the first grid point after the
  work ends. Waking slightly late from a sleep (OS scheduling, about a quarter of ticks by 0.5 ms or
  more in the September logs) is not an overrun: that slot runs, and its lateness is recorded.
- **`JointState.sequence` is the slot index**, not a count of reads, so a skipped slot is a
  sequence gap in the joint stream, and the log says exactly when it happened. The 30 Hz tick runs
  on every third or fourth slot of the same grid; if its slot is skipped, that `Tick` is skipped
  too, and shows up as a gap in the tick stream.
- Today's `Ticker` (`crates/control`) is already grid-based but starts a slot late when the
  work overruns into its window; Phase 1 changes it to this rule.

### Rates, as measured

From Part 1's hardware byte logs (`grab1`, `fast`, `fast2`; follower; about 2.3 minutes): the servo
loop runs at **100 Hz** (interval 10.0 ms typical, 14 ms p99). A sync read of six positions takes
1.0 ms typical, 3-4 ms p99; one servo's 15-byte telemetry block takes 0.7 ms. The leader is read at
the same rate. Ticks and policy actions run at 30 Hz on top, as the spec says: the servo layer
runs faster underneath. These are the driver's choices, not hardware limits, and can change.

Each tick's bus work (follower read, leader read, follower write, one telemetry read) takes 3 ms
typically and 7-8 ms at p99; about a quarter of ticks also wake more than 0.5 ms late, which is OS
scheduling rather than the bus. Those two together explain the 14 ms p99 interval.

**Open measurement: lock the servo loop to 90 or 120 Hz** (three or four reads per 30 Hz tick).
On the September logs, recorded on the Mac, 0.5-0.9% of ticks would overrun a 120 Hz slot
(8.3 ms) and 0.06-0.4% a 90 Hz slot (11.1 ms). The arms are now on the Spark, with a different
USB stack and scheduler, so measure there before choosing: run `so101 teleop --hz 120` with
telemetry on for a few minutes and count overruns. Under 1% means 120 Hz; otherwise 90 Hz. Nothing
in this design depends on the answer; it needs the arms plugged in and their ports in
`data/robot.conf`.

### Units and names everywhere

LeRobot's: degrees, gripper 0-100, joints `shoulder_pan` … `gripper`, cameras `wrist` and `scene`.
Export to a LeRobot dataset is then a straight copy.

## Behaviour under failure

| Failure | What happens | What it costs |
|---|---|---|
| **Slow or stalled disk** | The control loop always wins: publishing never blocks it. The recorder's queues are bounded. When it falls behind it **drops camera frames first**, and logs each drop (topic and sequence range) on `/log/drops` | Some frames, never the session. Small messages have a much larger queue; only if that fills too (the disk is effectively dead) does the recorder stop, mark the log incomplete, and report it |
| **Process crash** (error return or panic) | The MCAP writer finishes the file as it unwinds | Nothing |
| **Kill or power loss** | The file is cut short | At most the last 1 MB chunk. `so101 log recover` rewrites the rest so Python can open it |
| **A servo read overruns its slot** | The next slot is skipped and the loop resumes on the grid (contract 5) | One joint sample, visible as a sequence gap; the loop never drifts |
| **Dropped camera frames** | Detected as defined in contract 3 (sequence gap, or long capture interval) | Reported per camera by the reader and the converter; never silently passed through |
| **Mismatched model and stats** | The policy refuses to load (contract 4) | A failed start instead of a wrong-but-plausible run |

## Messages in Phase 0 (`so101.v1`)

Only what Phases 0-2 write (the runtime on fakes, then the policy):

- `JointState {t, pos[6]}`, 100 Hz: one sync read of all six positions
- `ServoTelemetry {t, joint, velocity, load, voltage, temperature}`: one per read. The driver reads
  one servo's telemetry block per tick, rotating, so each joint comes round at about 16.7 Hz with
  its own timestamp. A combined six-joint message would invent readings or blur their times. The
  spec's `JointState {pos, vel, current, temp}` is the logical view: a reader that wants it joins
  the latest telemetry per joint onto each position read
- `JointTarget {t, pos[6], source}`, 30 Hz. `source` is teleop, script, or a policy ID
- Frame: `foxglove.CompressedImage {timestamp = t_capture, frame_id = camera, data = jpeg, format = "jpeg"}`,
  so Foxglove plays both cameras as video. A frame's identity is `(camera, sequence)`
- `Tick {t, state[6], frame IDs (wrist, scene), action[6], source}`, 30 Hz
- `Chunk {t_obs, k, actions[k][6], ckpt_id}`: every predicted chunk
- `LogDrop {topic, first_sequence, last_sequence, count}`: what the recorder dropped

Arriving later, each with its phase: `RunStep`, `StepResult`, `CheckResult` and `Command` (Phase 3),
episode start/end with tags and `Takeover` (Phase 4), `SequencerDecision` (Phase 9).

## Clock and traits

- `Clock`: `now() -> Timestamp`, `sleep_until(Timestamp)`. `MonotonicClock` reads
  `clock_gettime(CLOCK_MONOTONIC)` (Rust's `Instant` can't give the raw value; macOS has the call
  too). `ReplayClock` is driven by a log's timestamps, at real speed or as fast as possible.
- Traits, all taking and returning domain types: `ArmDriver` (JointTarget in, JointState out),
  `FrameSource`, `TargetSource`, and `Policy` (Obs → Chunk, with contract 4 in its docs and its
  constructor). `Obs` is a Rust type; what the loop actually used is logged as `Tick`. `Skill`,
  `Check` and `Sequencer` arrive in Phase 3 with the step runner that calls them. Signatures are
  refined when their first implementation lands.

## Where logs come from

The arms and cameras are on the Spark, so real sessions are logged there, into `/projects/logs/`.
The laptop logs simulated runs only, into `data/logs/`, with synthetic or replayed cameras; the
header's host and clock source keep the two from being confused. Names follow
`<kind>-<YYYYMMDD-HHMMSS>.mcap`, so nothing is ever overwritten.

## Implementation

| | Choice | Why |
|---|---|---|
| Log container | MCAP, one file per session, one channel per topic, no compression | Indexed, self-describing, maintained Rust and Python readers, opens in Foxglove. JPEG doesn't compress, and leaving compression off drops the C dependencies |
| Encoding | Protobuf on every channel, frames as vendored `foxglove.CompressedImage` | Versioned messages shared by Rust and Python |
| Rust types | A thin domain layer in `contracts` over the generated types | Fixed `[f32; 6]` arrays and validation; traits never see generated types |
| Concurrency | Plain threads, blocking calls, no async | Matches the existing code; nothing needs async |
| LeRobot | Our log, plus a converter to LeRobotDataset v3 (`ml/export`, Phase 4) | LeRobot's format can't hold real timestamps, servo-rate state, the original JPEGs or async events, and its writer isn't crash-safe |

What we encode is only the log. Everything else is someone else's format, read with their tools:

| File | Format | Chosen by |
|---|---|---|
| Session logs (`*.mcap`) | MCAP, protobuf messages, JPEG frames | us |
| Datasets | Parquet, MP4, JSON (LeRobotDataset v3) | LeRobot |
| Checkpoints (`pretrained_model/`) | safetensors, JSON | LeRobot |
| Policy (`act.onnx`) | ONNX (itself protobuf, but `ort` reads it; it never touches our contracts) | ONNX |

```
proto/                     shared by every language in the repo
  so101/v1/*.proto             the messages
  foxglove/                    CompressedImage.proto, vendored from Foxglove's schemas (MIT)
  so101.v1.snapshot.binpb      the frozen schema the compatibility test checks against
crates/contracts/
  build.rs                     protox + prost-build: pure Rust, no protoc; embeds the descriptor set
  src/pb.rs                    generated types (only `log`, and later the PyO3 crate, use them)
  src/joint.rs, camera.rs,     one module per concept: its domain types, their protobuf conversions,
      tick.rs, policy.rs,      and the trait that produces or consumes them. A later phase adds
      log.rs                   modules (step.rs, check.rs, ...) rather than growing these
  src/time.rs                  Timestamp, MonotonicClock, ReplayClock
crates/session-log/
  bus, event (topics), recorder, raw (MCAP parsing), reader, drops, replay, recover
```

- **Bus:** in-process fan-out of `Event` (an enum over every message, with its topic). Publishing
  never blocks.
- **Recorder:** one thread owns the MCAP writer (it isn't `Sync`). Topics: `/follower/state`,
  `/follower/telemetry`, `/follower/target`, `/leader/target`, `/tick`, `/cam/wrist`, `/cam/scene`, `/policy/chunk`,
  `/log/drops`. Header as in contract 1, with the full config file as an attachment.
- **Reader:** a typed iterator of Events in time order, optionally by topic, which also reports
  drops (contract 3).
- **New dependencies:** `prost` and `bytes` (runtime), `protox` and `prost-build` (build only),
  `mcap` with default features off.

## Tests (written first)

1. Round trip: every message type, written and read back, gives identical records (f32 bits,
   sequence numbers and both timestamps included).
2. Drops: a camera stream with a sequence gap, and one with a long capture interval but no gap,
   are each reported, as different kinds.
3. Slow disk: with the writer stalled, the publisher never blocks, frames are dropped first, and
   every drop appears on `/log/drops`.
4. Replay timing: a recorded run replayed on `ReplayClock` reproduces its inter-event times.
5. Schema snapshot: the compiled descriptors match the frozen `so101.v1` snapshot.
6. Recovery: a file truncated mid-write recovers every complete chunk.
7. Python: a small `uv run` script reads a test log with `mcap` + `mcap-protobuf-support`, without
   generated code.

The policy's load-time check (contract 4) is tested in Phase 2, when the first policy exists.

## Out of scope for Phase 0

Moving today's teleop and byte log onto the bus (Phase 1), real cameras, the policy, the LeRobot
converter (Phase 4), and any change to the dashboard (its live feed stays JSON).
