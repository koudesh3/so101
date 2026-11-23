# Phase 0 design: contracts and log

Status: built, 2026-11-22. Implements Phase 0 of [part2-spec.md](part2-spec.md).

Built as designed, in `crates/so101` (reorganised on 2026-11-23 into one crate, one folder per
thing: see Implementation), with two differences: the reader loads a whole file into memory; and
recovery keeps whole messages from a truncated chunk too, so a cut file loses less than its last
chunk. "Contracts" names what this phase locked (the promises below and the log format), not a
place in the code.

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
  the host, the clock source, a monotonic + wall-clock pair read together at session start, the
  config, every calibration file (contract 6), and each camera's device and read-back settings
  (contract 7).
- Only messages something produces get defined. Adding a message later is a non-breaking change;
  a premature one would be frozen by the snapshot.

### 2. Tick is the truth about what the loop saw

The central promise. At every control tick (30 Hz) the loop logs one `Tick`: the state it used,
the frame it used from each camera (by identity), the action it sent, and the source of that
action. **Anything downstream (the LeRobot converter, analysis, probes) reads the loop's view of
the world only from `Tick` and the messages it references. Nothing re-derives it from timestamps.**
So a converted episode matches what the policy or operator actually acted on, by construction.

"Action" means one thing everywhere. In a demo the source is the leader arm; in a run it's the
policy; either can be clipped before it reaches the follower. So:
- **`Tick.action` is the target actually sent to the follower, after clipping**, in demos and runs
  alike. Training reads it, so a policy learns what was executed, not what was asked for.
- **`Tick.requested`** is what the source asked for, before clipping.
- **`Tick.clipped`** lists the joints where the two differ and which limit did it (the per-tick
  step limit, LeRobot's `max_relative_target`, or the joint's range). Clipping is then countable
  per episode: frequent clipping in demos means the step limit is fighting the operator, which is
  worth knowing before training on them.
- **Training reads `Tick.action`, never the leader's pose.** Teleop doesn't send the leader's pose
  as is: Part 1's teleop commands 40 ms ahead on pan, lift, elbow and gripper, extrapolating from
  the leader's velocity, because the follower trails by about 50 ms (measured below). So the lead
  belongs to the teleop source, before the shared clipping stage, never after it: a policy trained
  on `action` already outputs lead-adjusted targets, and a lead applied to its output would be
  applied twice. Keep that boundary when Phase 1 builds the mux.

### 3. What the timestamps and sequence numbers mean

- All times are u64 nanoseconds of the host's monotonic clock (`CLOCK_MONOTONIC`). It never jumps,
  unlike wall time under NTP. Times compare only within one log; real dates come from the header's
  monotonic/wall pair.
- Every MCAP record carries two times (MCAP's names). **`publish_time`** is when the thing
  happened: a frame's capture, a servo read, a decision. **`log_time`** is when our process
  received it, so `publish_time <= log_time`. The event time is also inside the message, so a
  message read alone is complete. Channels line up by `publish_time` (what the world looked like),
  replay orders by `log_time` (what the loop knew), and staleness is `log_time - publish_time`, all
  from the record headers alone.
- **`publish_time` is the best stamp we can get, not the physical instant.** Every source stamps
  slightly after the real event, with a bias that is small and steady for each source. That's
  fine, as long as the bias is known:
  - **Joint state:** stamped when the sync read returns. The servos sampled sometime in the
    ~1 ms round trip before that. Phase 1 stamps the midpoint of request and reply instead, which
    bounds the error to about ±0.5 ms against a 10 ms slot.
  - **V4L2 camera on Linux:** the kernel's buffer timestamp (same clock as `CLOCK_MONOTONIC`),
    which the UVC driver marks as start of exposure (`ts-src-soe`). Measured for the wrist camera
    (below): exposure actually starts about 10 ms before the stamp; no correction is applied yet. Each camera model needs
    its own check: the driver's claim depends on the camera's own clock reports.
  - **Synthetic camera:** when the frame was generated (exact). **Replayed camera:** the original
    capture time, offset onto the replay's clock.
  - **Ticks and targets:** the moment the loop made the decision (exact; we are the source).
- **Nothing may assume a camera delivers frames at its nominal rate.** The icSpring delivers
  29.35-29.8 fps, not 30, with intervals alternating between about 32.3 and 36.6 ms at any exposure
  (measured below). Numbering frames 0, 1, 2… as 33.3 ms apart drifts 0.4-1.2 s per minute. The
  converter places frames on its 30 Hz grid by `publish_time`, nearest frame per slot; the runtime
  uses a tick's latest frame by time. Pairing a frame with joint state happens in one function.
- Every record's **`sequence`** is a per-source counter set by the producer: the kernel's frame
  number for a camera, the producer's own count for joint state, targets and ticks.
- **A sequence gap proves the host lost something; it doesn't prove the camera didn't skip a
  frame.** The V4L2 counter counts frames the driver received, not frames the camera never sent.
  That was the icSpring's failure in the camera spike: under auto exposure it fell to 16-20 fps with no
  gaps in its counter. So a drop is defined as **a sequence gap, or a capture interval longer than
  1.5 × the camera's nominal period**. The reader and the converter report both kinds, separately.

### 4. What a policy guarantees

- A `Policy` takes and returns **degrees** (gripper 0-100), exactly the units of `JointState` and
  `JointTarget`. Nothing outside the policy ever sees normalized values.
- A policy **owns its normalization.** The spec's export puts it inside the ONNX graph; if it ever
  lives in a separate stats file instead, that file (JSON) names the model it belongs to.
- **Not true yet:** the ONNX spike's graph (`/projects/spikes/act-onnx`) takes and returns
  normalized values, because LeRobot 0.7 keeps normalization in its processor pipeline, outside
  `ACT`, and the spike's parity test compared normalized numbers. Phase 2's export must wrap the
  network with the normalize and unnormalize steps, and its parity test must compare **degrees in
  to degrees out**, so the check covers the whole path.
- A policy **refuses to load** if its normalization doesn't match its model, so a model with the
  wrong stats (which moves the arm plausibly and wrongly) can't run.
- Every `Chunk` carries the checkpoint ID that produced it.

### 5. The servo loop keeps to its grid

- Slot *n* starts at `t0 + n × period`, a fixed grid (120 Hz, see below), never one period
  after the previous work finished, so the loop can't drift.
- **An overrun skips, never starts late.** If slot *n*'s work is still running when slot *n+1*
  should start, slot *n+1* is skipped, and the loop sleeps until the first grid point after the
  work ends. Waking slightly late from a sleep (OS scheduling, about a quarter of ticks by 0.5 ms or
  more in the September logs) is not an overrun: that slot runs, and its lateness is recorded.
- **`JointState.sequence` is the slot index**, not a count of reads, so a skipped slot is a
  sequence gap in the joint stream, and the log says exactly when it happened. The 30 Hz tick runs
  on every third or fourth slot of the same grid; if its slot is skipped, that `Tick` is skipped
  too, and shows up as a gap in the tick stream.
- Today's `Grid` (`control/grid.rs`) is already grid-based but starts a slot late when the
  work overruns into its window; Phase 1 changes it to this rule and moves it onto `Clock`.

### Rates, as measured

From Part 1's hardware byte logs (`grab1`, `fast`, `fast2`; follower; about 2.3 minutes): the servo
loop runs at **100 Hz** (interval 10.0 ms typical, 14 ms p99). A sync read of six positions takes
1.0 ms typical, 3-4 ms p99; one servo's 15-byte telemetry block takes 0.7 ms. The leader is read at
the same rate. Ticks and policy actions run at 30 Hz on top, as the spec says: the servo layer
runs faster underneath. These are the driver's choices, not hardware limits, and can change.

Each tick's bus work (follower read, leader read, follower write, one telemetry read) takes 3 ms
typically and 7-8 ms at p99; about a quarter of ticks also wake more than 0.5 ms late, which is OS
scheduling rather than the bus. Those two together explain the 14 ms p99 interval.

**Decided: the servo loop runs at 120 Hz** (four slots per 30 Hz tick). Measured on the Spark
on 2026-11-23: `so101 teleop --hz 120` with telemetry on, arms moved by hand including fast moves,
17,486 ticks (2.4 minutes). Bus work per tick was 3.7 ms typical and 5.7 ms p99, against an 8.3 ms
slot: 0 overruns (the one 11.8 ms tick was the Ctrl-C that ended the run). 16% of ticks woke more
than 0.5 ms late, which doesn't skip a slot. On the September logs, recorded on the Mac, 0.5-0.9%
of ticks would have overrun at 120 Hz; the Spark's USB stack and scheduler are faster. Byte log:
`/projects/logs/hz120-20261123-133445.jsonl` on the Spark.

**Measured: the wrist camera and the 120 Hz loop don't disturb each other.** Same day, 90 s of
teleop at 120 Hz with the wrist camera streaming 640x480 MJPG on the same Spark: bus work per tick
4.5 ms typical, 6.8 ms p99; 2 of 10,800 ticks (0.02%) ran over the 8.3 ms slot (the current
`Ticker` didn't count them as overruns; Phase 1's skip rule will). The camera delivered 2,709
frames at 29.8 fps with no sequence gaps and no interval over 1.5x the median (intervals 32.3 ms
typical, 36.6 ms at most). Leader to follower lag, from cross-correlating joint velocities: pan
42 ms, lift 38 ms, elbow 46 ms (these three have the 40 ms teleop lead), wrist flex 70 ms, wrist
roll 68 ms (no lead), gripper 106 ms. That's the ~50 ms servo dead time plus the loop. Data:
`/projects/spikes/cam-offset/teleop-135403/` on the Spark.

**Measured: the wrist camera's exposure starts about 10 ms before its kernel stamp** (contract 3).
On the Spark, 2026-11-23, the wrist camera (the icSpring) at 640x480 MJPG, manual exposure 30 ms, frame
period 32.2 ms. A script held the follower still and stepped wrist flex by 10 degrees every 2 s,
logging joint positions (600 reads/s, stamped at the request/reply midpoint) and the camera's kernel
timestamps from `v4l2-ctl`. Fitting the image change between frames against the wrist's movement
between their exposure windows peaks sharply with exposure starting at `ts - 8 ms` (r 0.96); a
second run stepping wrist roll by 30 degrees, which turns the camera itself, gave `ts - 12 ms`
(r 0.97). Call it 10 ms, ±5. So a frame stamped `ts` shows the arm as the joint log has it over
`ts - 10` to `ts + 20 ms`, centred about 5 ms after `ts`. That's relative to the joint log's clock, which is what alignment
needs. An earlier run with the arm moved by hand only bounded it within a frame (hand moves are too
smooth); the driver labels its stamps start of exposure (`ts-src-soe`), which is close but not
exact. **Decision:** the offset is under a quarter of a frame, so the converter uses the kernel
stamp as is for now; if Phase 5 parity needs it, subtract 10 ms. The scene camera isn't a policy
input, so it doesn't need this check. The spike lives at `/projects/spikes/cam-offset` on the Spark
(`step.py`, `step_analyze.py`, `step_fit.py`, runs `step-134517/` and `roll-134728/`; the hand-moved run is `run-133941/`).

**Measured: the follower takes about 50 ms to start moving after a goal write, more under load.**
Same step tests, with acceleration at maximum (254), no speed limit, P 32, D 32, I 0, measured from
writing the goal to the reported position moving 3 ticks. Wrist roll, which carries no gravity
load: 50.6 ms median, 49.6-53.2 ms, and a 30 degree step settles within 5 ticks in 295 ms. Wrist
flex, 10 degree steps: 58-72 ms; steps down settled in 170 ms, steps up, against gravity, stopped
short of 5 ticks (no integral term). Whether that's the motor or
a delay in the reported position, it's what the loop sees: an action shows up about two camera
frames later. It probably explains why Part 1's 40 ms teleop lead helped. A policy trained on
teleop data learns this delay implicitly; the runtime shouldn't try to hide it.

**Measured: the follower's gains need no tuning, only pinning.** 2026-11-23, follower held in a
reaching pose (upper arm about 45 degrees up, forearm level, so gravity loads every joint), 10
degree steps up and down on wrist flex, elbow and shoulder lift, at P/I of 32/0, 32/1, 32/2, 32/4,
32/8 and LeRobot's 16/0 (D 32 throughout):
- At the factory gains (P 32, I 0, D 32, unchanged on every servo) every joint stopped within
  0.4-0.7 degrees of its goal. I gain barely helps (about 0.2 degrees at I 8) and adds up to
  0.8 degrees of overshoot. LeRobot's P 16 is 60-250 ms slower and stops up to 2.3 degrees short.
- **Shoulder lift overshoots about 1 degree and wobbles at roughly 5 Hz for about 400 ms** before
  settling, at every gain tried. Teleop shows the same 5 Hz content (below), so it's the likely
  source of the arm shaking on fast moves. A short D sweep on shoulder lift is the one tuning
  question left.
- The 5.5 degree stall on upward wrist-flex steps (the first step test) happens only in the folded
  pose; the same joint at the same gains reaches 96-100% in the reaching pose. Something in that
  pose resists the wrist (the camera cable, or contact); check it by eye before trusting folded
  poses as waypoints.
- About 1 in 2,500 temperature readings is junk (44 to 150 degrees C in a steady 31-36 band), in
  packets with a correct checksum and servo ID, so the servo itself sends them. A median of each
  joint's last 5 readings removed every false reading over 55 C in 27,009 samples; a fixed ceiling
  did not (64, 66 and 77 C passed). See contract 8.
- Spike: `/projects/spikes/cam-offset/tune-reach-141514/` on the Spark (`tune.py`, `tune_analyze.py`).

**Measured: target jitter from the teleop lead, and follower shake.** On the 90 s teleop run, the
5-30 Hz content of the target sent on joints with the lead is 1.2-2.1 times the leader's (pan,
lift, gripper), because the lead extrapolates from velocity, the noisiest signal derived from
position reads; on the wrists, without a lead, target and leader match exactly. The elbow's worst
second moved 0.40 degrees in that band against 0.17 degrees of target. Detecting shake therefore
compares each joint's follower motion with its target in the same band; it needs only the 120 Hz
position stream, no extra bus reads.

### 6. Degrees are only meaningful with their calibration

LeRobot turns a raw servo reading into degrees from the midpoint of that motor's calibrated range,
so the same pose reads as different degrees after a recalibration (after a crash or a
re-assembly), and older logs would describe slightly wrong poses relative to the new runtime.
- Every log stores **each calibration file itself** (as the attachment `calibration/<name>.json`)
  and **its SHA-256 in the header**. The reader refuses a log whose file doesn't match its hash.
- `JointState` and `JointTarget` carry the **raw servo ticks** next to the degrees, so any log can
  be recomputed under a different calibration.
- The converter refuses to mix calibrations in one dataset unless told to convert (Phase 4).

### 7. A policy runs at the camera settings it was trained at

The wrist camera (the icSpring) forgets its settings when unplugged; `act record` re-applies them, and the runtime must
too, or a policy trained at one exposure runs at another.
- Every log's header stores, per camera role, **the physical device** (its `/dev/v4l/by-id/` path
  or serial) and **the settings read back from it** at session start, in recordings and runs
  alike. Tying settings to the device makes a swapped wrist/scene config visible instead of
  silently applying one camera's settings to the other.
- The runtime refuses to start if a camera's read-back settings differ from the training data's
  (Phase 4, when the runtime first drives real cameras).

### 8. A policy runs at the servo gains it was trained at

Servo gains and limits live in each servo's memory and survive power cycles, and every tool that
connects writes its own: Part 1's teleop writes P 32; LeRobot's follower writes P 16, I 0, D 32 on
every connect, and caps the gripper at Max_Torque 500, Protection_Current 250, Overload_Torque 25
(ours: 800). So the follower's stiffness depends on which tool ran last, and nothing records it.
- **One file in the repo holds the follower's gains and limits**: P 32, I 0, D 32 (measured
  above), and the gripper at LeRobot's limits (Max_Torque 500, Protection_Current 250,
  Overload_Torque 25), decided 2026-11-23. 50% closes more slowly than Part 1's 80% but heats less
  while holding a disc through a transit, and matches anything recorded with `act` before Phase 4.
  Revisit only if a gripper test with the real discs shows slipping. Our
  runtime applies it on every start and reads it back; `act` passes the same values to LeRobot
  (`--robot.position_p_coefficient=32` and the I and D equivalents).
- **Every log's header stores the gain and limit registers read back from each servo** at session
  start, like calibration and camera settings.
- The runtime refuses to run a policy whose training data's gains differ (Phase 4).
- Servo health checks (temperature, voltage) act on the median of a joint's last 5 readings, never
  a single reading; the log keeps every raw reading, and the reader counts impossible values
  (temperature over 100 C) as a bus-health signal.

### Units and names everywhere

LeRobot's: degrees, gripper 0-100, joints `shoulder_pan` … `gripper`, cameras `wrist` and `scene`.
Export to a LeRobot dataset is then a straight copy, given one calibration (contract 6).

## Behaviour under failure

| Failure | What happens | What it costs |
|---|---|---|
| **Slow or stalled disk** | The control loop always wins: publishing never blocks it. The recorder's queues are bounded. When it falls behind it **drops camera frames first**, and logs each drop (topic and sequence range) on `/session/drops` | Some frames, never the session. Small messages have a much larger queue; only if that fills too (the disk is effectively dead) does the recorder stop, mark the log incomplete, and report it |
| **Process crash** (error return or panic) | The MCAP writer finishes the file as it unwinds | Nothing |
| **Kill or power loss** | The file is cut short | At most the last 1 MB chunk. `so101 session recover` rewrites the rest so Python can open it |
| **A servo read overruns its slot** | The next slot is skipped and the loop resumes on the grid (contract 5) | One joint sample, visible as a sequence gap; the loop never drifts |
| **Dropped camera frames** | Detected as defined in contract 3 (sequence gap, or long capture interval) | Reported per camera by the reader and the converter; never silently passed through |
| **Mismatched model and stats** | The policy refuses to load (contract 4) | A failed start instead of a wrong-but-plausible run |
| **The arm is recalibrated** | New logs carry the new calibration and its hash; raw ticks are logged throughout (contract 6) | Nothing lost: old logs can be recomputed; the converter won't silently mix them |
| **Camera settings drift, or cameras swapped** | Each log records device and read-back settings; the runtime refuses to start on a mismatch (contract 7) | A failed start instead of a policy running at the wrong exposure |
| **Servo gains changed by another tool** | Each log records the gains read back from every servo; the runtime applies the repo's gains on start and refuses a policy trained at others (contract 8) | A failed start instead of a policy running on a stiffer or softer arm |
| **A servo reports a junk value** | Health checks use the median of the last 5 readings; the raw value stays in the log and is counted (contract 8) | Nothing: no false stop, and the glitch rate is visible |
| **A target is clipped** | `Tick.action` is what was sent, `Tick.requested` what was asked, `Tick.clipped` which joints and why (contract 2) | Nothing hidden: clipping is countable per episode |

## Messages in Phase 0 (`so101.v1`)

Only what Phases 0-2 write (the runtime on fakes, then the policy):

- `JointState {t, pose[6], ticks[6]}`, 120 Hz: one sync read of all six positions, as a pose and
  as the raw servo ticks behind it (contract 6)
- `ActuatorStatus {t, joint, velocity, load, voltage, temperature, current, faults[]}`: one per
  read. The driver reads one actuator per slot, rotating, so each joint comes round with its own
  timestamp. A combined six-joint message would invent readings or blur their times. `faults` are
  decoded by the driver (overheat, overload, ...), so nothing above it reads a servo register
- `JointTarget {t, pose[6], source, ticks[6]}`, 30 Hz. `source` is teleop, script, or a policy ID;
  `ticks` are the raw goals actually written
- Frame: `foxglove.CompressedImage {timestamp = t_capture, frame_id = camera, data = jpeg, format = "jpeg"}`,
  so Foxglove plays both cameras as video. A frame's identity is `(camera, sequence)`
- `Tick {t, slot, state[6], frame IDs (wrist, scene), action[6], source, requested[6], clipped[]}`,
  30 Hz (contract 2). `slot` is the servo slot whose `JointState` it used and in which its target
  was written, so a tick can be recomputed from raw ticks under another calibration
- `Chunk {t_obs, period, k, actions[k][6], checkpoint_id}`: every predicted chunk, with the time
  between its actions stated rather than assumed
- `RecorderDrop {topic, first_sequence, last_sequence, count}`: what the recorder dropped

Arriving later, each with its phase: `RunStep`, `StepResult`, `CheckResult` and `Command` (Phase 3),
episode start/end with tags and `Takeover` (Phase 4), `SequencerDecision` (Phase 9).

## Clock and traits

- `Clock`: `now() -> Timestamp`, `sleep_until(Timestamp)`. `MonotonicClock` reads
  `clock_gettime(CLOCK_MONOTONIC)` (Rust's `Instant` can't give the raw value; macOS has the call
  too). `ReplayClock` is driven by a log's timestamps, at real speed or as fast as possible.
- **A trait arrives when something needs it, not before.** `Arm` exists because the control loop
  must reach arms only through it (teleop is its first user; the mock arm joins `ServoArm` in
  Phase 1); the camera trait with the replay camera; the policy trait
  with Phase 3's stubs beside the ONNX policy, carrying contract 4 in its docs and constructor.
  Each lives in its thing's folder.

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
| Rust types | Domain types in each thing's folder; protobuf only inside `session/wire.rs` | Fixed `[f32; 6]` arrays and validation; nothing outside the log sees a generated type |
| Concurrency | Plain threads, blocking calls, no async | Matches the existing code; nothing needs async |
| LeRobot | Our log, plus a converter to LeRobotDataset v3 (`ml/export`, Phase 4) | LeRobot's format can't hold real timestamps, servo-rate state, the original JPEGs or async events, and its writer isn't crash-safe |

What we encode is only the log. Everything else is someone else's format, read with their tools:

| File | Format | Chosen by |
|---|---|---|
| Session logs (`*.mcap`) | MCAP, protobuf messages, JPEG frames | us |
| Datasets | Parquet, MP4, JSON (LeRobotDataset v3) | LeRobot |
| Checkpoints (`pretrained_model/`) | safetensors, JSON | LeRobot |
| Policy (`act.onnx`) | ONNX (itself protobuf, but `ort` reads it; it never touches our log format) | ONNX |

```
proto/                     shared by every language in the repo
  so101/v1/messages.proto      the messages
  foxglove/                    CompressedImage.proto, vendored from Foxglove's schemas (MIT)
  so101.v1.snapshot.binpb      the frozen schema the compatibility test checks against
crates/so101/              one crate, one folder per thing
  src/arm/                     Joint, Pose, JointState, JointTarget, ActuatorStatus, the `Arm` trait
                               and `ServoArm` (mod.rs); calibration; the STS3215 servos and their
                               registers (servos.rs); the UART; simulated servos
  src/camera/                  Frame, FrameRef (the V4L2 code joins it in Phase 4)
  src/policy/                  Obs, Chunk (the ONNX runner joins it in Phase 2)
  src/control/                 the loop: grid (slots), Tick, events, bus, teleop
  src/session/                 a recorded session: wire (protobuf conversions, generated from a field
                               map), recorder, reader, drops, replay, recover
  src/clock.rs                 Timestamp, Clock, MonotonicClock, ReplayClock
  src/bin/so101.rs             the command line: puts the pieces together
  build.rs                     protox + prost-build: pure Rust, no protoc; embeds the descriptor set
```

`just check` enforces the one layering rule: `control/` reaches arms only through the `Arm` trait,
never `ServoArm`, the servos, the UART, the simulated servos, calibration or the session.

- **Bus:** in-process fan-out of `Event` (an enum over every message, with its topic). Publishing
  never blocks.
- **Recorder:** one thread owns the MCAP writer (it isn't `Sync`). Topics: `/follower/state`,
  `/follower/actuators`, `/follower/target`, `/leader/target`, `/tick`, `/cam/wrist`, `/cam/scene`, `/policy/chunk`,
  `/session/drops`. Header as in contract 1, with the full config file as an attachment.
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
   every drop appears on `/session/drops`.
4. Replay timing: a recorded run replayed on `ReplayClock` reproduces its inter-event times.
5. Schema snapshot: the compiled descriptors match the frozen `so101.v1` snapshot.
6. Recovery: a file truncated mid-write recovers every complete chunk.
6b. Calibration: a log whose calibration file doesn't match the hash in its header is refused.
7. Python: a small `uv run` script reads a test log with `mcap` + `mcap-protobuf-support`, without
   generated code.

The policy's load-time check (contract 4) is tested in Phase 2, when the first policy exists.

**Tests that belong to later phases, written into the spec:**
- Phase 2: export parity in degrees (contract 4), and the image boundary test. At run time the
  policy gets the camera's JPEG decoded in Rust; in training it gets LeRobot's MP4 re-encode
  decoded in Python. Run one log through both paths and compare the tensors: channel order (RGB
  vs BGR), value range (0-255 vs 0-1) and size/resize must match exactly; the pixel difference left
  over is the compression's, measured against a tolerance (it can't be zero, the MP4 is lossy).
- Phase 4: the converter refuses mixed calibrations (contract 6); the runtime refuses mismatched
  camera settings (contract 7).

## Out of scope for Phase 0

Moving today's teleop and byte log onto the bus (Phase 1), real cameras, the policy, and the
LeRobot converter (Phase 4). Part 1's web dashboard was removed on 2026-11-23: viewers read the
session log (Foxglove now; a live stream, if wanted, later).
