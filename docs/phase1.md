# Phase 1 design: the control loop, on fakes and the real rig

Status: design 2026-11-23; revised 2026-11-24 after a day of teleop, recording and two analysed
episodes on the real arms (see "What 2026-11-24 changed"). Implements Phase 1 of
[part2-spec.md](part2-spec.md), on the contracts of [phase0.md](phase0.md). Names follow the
glossary in the [README](../README.md).

Phase 1 builds the loop that every later phase runs inside: one loop for teleop, scripts and
policies, at fixed rates, with safety in it and every decision recorded. It runs on a mock arm and
synthetic or replayed cameras, so it needs no hardware, and on the real arms and cameras unchanged.

## The loops

A robot is nested loops, each at its own rate. Each one hands a setpoint down to the faster loop
below it and gets observations back. The code follows the same layers.

| Loop | Rate | Decides | Hands down | Gets back | Code |
|---|---|---|---|---|---|
| Task | per Hanoi move, ~0.05 Hz | which disc goes where next | a subtask | subtask results | `runtime/` (Phase 3) |
| Subtask | 2-10 s | run a skill (grasp, place, transit) until its check says done or failed | a policy or script | check results | skills, checks (Phase 3) |
| Action chunk | each inference, ~1-3 s | the next stretch of motion | a `Chunk` of poses | an `Obs` | `policy/` (Phase 2) and the action queue |
| **Tick** | **30 Hz** (the camera) | this tick's target, within safety | a target `Pose` | `JointState`, latest `Frame` | `control/` |
| **Cycle** | **120 Hz** (the grid) | the goal to write now, stall holds, slew limit | goal `Counts` | `Counts`, faults | `control/` and `arm/` |

Phase 1 builds the two loops in bold, and the action queue at the boundary with the action chunk
loop. Below the cycle, each STS3215 runs its own position loop in firmware; it isn't ours,
and only its gains are (contract 8).

**Rules.**
1. **A slower loop never blocks a faster one.** Inference runs on its own thread and hands chunks
   to the action queue; the tick takes whatever is queued and never waits. Checks on frames
   (Phase 3) work the same way, and so do cameras, the recorder and the console.
2. **Each loop talks only to its neighbours.** The tick never sees servos (the `Arm` trait is the
   boundary, enforced by `just check`); the sequencer never sees ticks.
3. **Safety lives at the fastest loop that can see the danger.** Stall holds and the slew limit
   need counts and faults on every write, so they run per cycle. Joint ranges are clipped per tick.
   A wrong plan is caught by checks at the subtask loop.
4. **Setpoints go down and observations come up, in glossary types:** subtask → `Chunk` → target
   `Pose` → goal `Counts`; `Counts` → `JointState` → `Tick` and `Obs` → check result.

## Decisions

- **One thread runs both cycles and ticks.** The grid runs at 120 Hz; every 4th cycle is also a tick
  (120 / 30). One thread keeps the timing deterministic, needs no locks, and lets every timing test
  run on a replayed `Clock`. Two threads would only earn their keep if tick work overran a cycle;
  inference is already off-thread.
- **Cycles step linearly toward the tick's target.** A tick sets a target; the four cycles up to the
  next tick move the goal in equal steps from the previous target to the new one. Smoother than
  holding the target for four cycles (LeRobot's behaviour), and it means the teleop lead is
  extrapolated 30 times a second and smoothed, not 120 times a second on noisy velocity: the
  measured cause of target jitter (phase0.md).
- **Ensembling, when used, lives in the action queue at the tick**, the only place that sees
  overlapping chunks. The default stays the spec's: play `n_action_steps` of each chunk, ensembling
  off, as `lerobot-eval` does.
- **`Tick.clipped` covers its four cycles:** a joint is listed if any of its cycles was clipped,
  with the most severe reason (`Stall`, then `Collision`, then `Step`, then `Range`).
- **Real V4L2 capture moves into Phase 1** (decided 2026-11-24, from Phase 4). Teleop already
  records real cameras, and demos are about to be recorded for training; capture through ffmpeg
  can't give the kernel's capture timestamps or frame numbers that contract 3 requires. The camera
  trait arrives with three implementations: V4L2, synthetic and replay. `FfmpegCamera` is retired
  once V4L2 lands.
- **The scene camera is stored beside the session, not in it** (decided 2026-11-24): an H.264 MKV
  plus a CSV of each frame's sequence and times on the session clock, next to the MCAP. The wrist
  camera, the only policy input, stays in the MCAP as the camera's own JPEGs. Scene JPEGs at
  1280x720 are about 22 GB an hour; H.264 is about 1.5 GB. Contract 1 and the Python check change
  to match (phase0.md).
- **Training reads `Tick.action`**, the target as sent, after every limit (contract 2). The
  exporter (Phase 4) takes it from there, never from the leader's pose.

## What gets built

All in `crates/so101/src/control/` unless noted. Every part works through the `Arm` trait, so it
runs on `ServoArm` and the mock arm alike.

**Already in place.** From the refactor before Phase 1 (2026-11-23):
- **One `Clock`.** The grid, `ServoArm`'s timestamps, the simulated servos and the loop all take
  their time from it. Tests run everything on one stepped `ReplayClock`, so they're exact.
- **The `Arm` trait speaks poses.** `command(target)` returns `Sent`: what was written and, per
  joint, which `Limit` changed it. `ServoArm` owns its protections in the counts they were tuned
  in. The loop never sees counts, faults or calibration.
- **`run` and `Pilot`.** `control::run` drives a follower on the grid: each cycle it reads the
  follower (and, with `read_actuators`, every servo's load, voltage, temperature and current),
  asks the `Pilot` for a target, commands it, and publishes `JointState`, `ActuatorStatus`,
  whatever the pilot observed, the `JointTarget` and the `Tick` on the `EventBus`. `Teleop` is the
  first pilot.
- **The grid's overrun rule** (contract 5), with lateness counted: `RunStats` counts skipped cycles
  and cycles that started more than 1 ms late. `sleep_until` spins the last 1 ms, and the loop
  runs at `SCHED_FIFO` 40 on the Spark.

From 2026-11-24:
- **Teleop records sessions.** Teleop owns both cameras and serves the console on 127.0.0.1:8090
  (`console.rs`; `just console` tunnels it to the laptop): both live views (the wrist view turned
  90° clockwise to match its mounting; recordings keep the camera's orientation), the follower's
  per-joint load, a contact tone, and record/stop. A recording is `data/logs/<date-time>.mcap`
  (every event and the wrist camera, through `session::Recorder`) with the scene camera's
  `-scene.mkv` and `-scene.csv` beside it. The header carries the commit, teleop's settings, both
  calibration files and each camera's device and read-back settings. `just rig` starts it all on
  the Spark from `/projects/robot.conf`.
- **Cameras through ffmpeg** (interim, removed the same day): frames stamped on arrival and
  numbered by us, which contract 3 doesn't allow. Replaced by V4L2 (below).
- **Gripper torque 30%** by default (it was 50%; see "What 2026-11-24 changed").

Phase 1 builds on that:

**Ticks every 4th cycle, with stepping** (30 Hz): on a tick, take the latest `JointState` and each
camera's latest `Frame`, record their identities in `Tick.wrist` and `Tick.scene`, and ask the
pilot for a target. On the cycles between, command a goal stepped linearly from the previous
target toward it. `Tick.clipped` collects its cycles' limits, most severe first. Today every cycle
writes a `Tick` with no frames referenced, so an exporter would have to pair frames by time
(contract 2 forbids it).

**Pilots**, one interface (`Pilot`): given the follower's state, a target `Pose` and what was
observed to decide it.
- **Teleop** (exists): reads the leader `Arm` and returns its pose plus the lead. Phase 1 filters
  the lead's velocity (it multiplied target jitter 1.2-2.1× when computed per cycle); the lead
  belongs here, before the arm's limits, never after (contract 2).
- **Script:** a list of waypoints, stored as targets (what was commanded), each with a duration.
- **Action queue:** turns `Chunk`s into one target per tick. Asks for the next chunk early enough
  that inference finishes before the current one runs out; holds the last target if it hasn't.
  Optional ensembling.

**The mux:** exactly one pilot is active. Switching is explicit (`Tick.source` changes on that
tick), and a newly active pilot starts from the follower's present pose, so nothing jumps.
Takeover from the leader (Phase 4) is a mux switch.

**Safety, by loop:**

| Check | Where | Action |
|---|---|---|
| Joint range | `ServoArm`, every write (exists) | clamp to the calibrated range (`Limit::Range`) |
| Step limit | `ServoArm`, every write (exists) | limit each goal's step from the last (`Limit::Step`) |
| Stall or overload | `ServoArm`, every write (exists) | hold at the contact point; the gripper keeps a small squeeze (`Limit::Stall`). On the gripper this is now the grip-force regulator: in the 2026-11-24 episode it held every grasp (6 holds, 1.5-5 s) at about 21% load instead of the 30% ceiling, and released as soon as the leader opened past about 7% |
| Drive ceiling | servo firmware, set at connect (exists) | Max_Torque per joint at about 1.2x the most Hanoi moves used (pan 90%, lift 80%, elbow 70%, wrist flex 80%, roll 70%) and the gripper at 30%; in EEPROM so a brownout reboot keeps it |
| Wrist camera vs arm | `ServoArm`, every write (exists) | a goal that brings the camera within 10 mm of the arm, and closer than the last goal, keeps elbow, wrist flex and wrist roll where they were (`Limit::Collision`). Clearance comes from `data/camera_clearance.json`, precomputed from the SO-101 model by `tools/camera_clearance.py`. Shoulder and base contact isn't covered |
| Supply current | `ServoArm`, every write (exists) | summed follower current over 1.5 A for 50 ms cuts the step limit to 10 counts until 250 ms under (`Limit::Step`). Teleop with the 30% gripper peaks at 0.67 A |
| Actuator health | cycle (exists, `control::Health`) | act on the median of each joint's last 5 temperature and voltage readings, judged only on a full window; count impossible values (contract 8). Stops with torque off at a median of 65 C (the servos cut out at 70 C) or a supply median under 85% of what it read at start (about 10.1 V on 11.9 V); readings over 10 C or 0.5 V from their median are counted as junk and reported when teleop stops. Both kinds of junk are real: on 2026-11-24 about 1 in 500 temperature readings was 45-150 C in a steady 34-39 C band (5x the earlier estimate), and the wrist roll and gripper servos alone reported single-sample voltages of 10.2-11.2 V at near-zero current, never two servos together, while the others read 11.8-11.9 V. The console's power line and status show the median too, not the minimum |
| E-stop | cycle | torque off on the next cycle; the session records why |
| Follower failing | cycle (exists) | after 0.5 s of failed cycles, torque off and stop |

**Shake measure.** Per joint, the follower's motion in the 5-30 Hz band against its target's in
the same band, from the 120 Hz `JointState` stream. Follower moving more than it was told to is
shake. Logged per tick window as a health number; no action in Phase 1.

**The mock arm** (`arm/`, implements `Arm`): follows goals the way the measurements say the
follower does: about 50 ms of dead time (more under gravity), then a first-order approach with a
speed limit, and a reversal dead band (0.3-0.4° on most joints, 1.7° on the elbow, measured
2026-11-24). Fault injection: stall at a position, overload flag, a joint that stops answering.
Fast and deterministic on a replayed clock; `--sim` can switch to it. `SimulatedServos` stays for
testing the servo driver byte for byte.

**Cameras** (`camera/`; done 2026-11-24), the `Camera` trait (role, nominal period, device and
read-back settings, blocking `next_frame`), `stream` to run one on its own thread counting
contract 3's drops (sequence gaps and long intervals, apart), and `LatestFrame`, a lock-free slot
the real-time loop reads without waiting. Three implementations, one conformance suite
(`tests/cameras.rs`):
- **V4L2** (`V4l2Camera`, Linux, the `v4l` crate): MJPEG through the kernel's mmap buffers;
  `t_capture` is the kernel's CLOCK_MONOTONIC stamp, `t_receive` ours, `sequence` the kernel's
  frame number. Controls by v4l2-ctl's names (robot.conf unchanged), auto switches first, read
  back; one that didn't take refuses the open. Two quirks worked around: the crate's own timeout
  re-queues a buffer the kernel holds (EINVAL), so we poll ourselves; the icSpring pads a byte
  after the JPEG's end marker, so frames are trimmed to it. The first 2 frames after start arrive
  truncated and are skipped. Measured on the Spark: both cameras 3 minutes together, 0 frames
  lost, 0 long intervals (wrist 29.8 fps, scene 29.5 fps); one wrist frame lost in an earlier
  10 s run, so losses happen but rarely.
- **Synthetic** (`SyntheticCamera`): exact times on any clock, chosen losses; `--sim` uses it.
- **Replay** (`session::ReplayCamera`): one role's frames from a session at their recorded times,
  with the recorded device and settings. Replay from LeRobot dataset video waits for the exporter.

**The scene video** (`scene_video.rs`; done 2026-11-24): saved raw while recording (the camera's
JPEGs in `<name>-scene.mjpeg`, a CSV of each frame's sequence, capture and receive times and byte
range), so the live path never waits on another process. Stopping a recording starts `so101
encode-scenes` as its own low-priority process: H.264 with every frame at its CSV capture time
(ffmpeg's concat input with per-frame durations at a 1 ms time base), checked frame by frame
within 1.5 ms before the raw file is removed; a recording cut short keeps its complete frames.
`just encode-scenes` finishes any left raw.

**The session:** the recorder keeps the scene camera beside the MCAP, with its CSV on the session
clock. Done 2026-11-24: one gains file, `data/gains.json` (P 32, I 0, D 32, per-joint
Max_Torque, gripper 30%), written on every start and read back, refusing to run if a servo
differs; every header records each servo's registers as read and attaches the file; `--gains`
replaces the `--p-gain` and `--grip-torque` flags; `act` passes the file's P/I/D to LeRobot (its
gripper stays at LeRobot's hardcoded 50%).

**Smaller changes:**
- `JointState.t` is the midpoint of request and reply, not the reply.
- The startup error (done 2026-11-24): the first cycle after connect sometimes failed ("reply
  from servo 2, expected 1", several runs). The byte trace showed why: servo 1 missed the first
  sync read, 0.8 ms after torque came on, so servo 2's reply read as the wrong servo. Torque-on now
  reads until all six answer (up to 100 ms) before the loop starts. Also, a read interrupted by a
  signal (Ctrl-C's) retries instead of counting as an error.

## What 2026-11-24 changed

A day of teleop on the real rig, a fine-motion measurement, two analysed episodes
(`20261124-200321`, 203 s, and `20261124-202319`, 158 s) and the first recordings.

- **Gripper force was the cause of discs flying, and 30% fixes it.** At 50% the gripper flung
  discs. In every grasp the operator closes the leader against its stop (0.5%) while the jaw stops
  at the disc (3.5-6%), so grip force is set by the torque ceiling, not the hand. At 30% with the
  stall hold, nothing flew in the second episode. The one ring that flew in the first left
  because it snagged, not because of the squeeze: the jaw opened and the shoulder lifted in the
  same 0.1 s, the lift's load jumped from 6% to 22%, and the ring sprang off the peg.
- **Load alone doesn't mean holding.** A fast open or close of an empty gripper also hits the
  ceiling (the 30% jaw trails the leader by 0.2-0.3 s). A holding check (Phase 3) uses the jaw's
  gap to its goal and the stall hold, not load.
- **Lag in real use is 60-80 ms** from leader to follower on every joint (cross-correlation over
  whole episodes), about twice the 33-42 ms of the slow-move test.
- **Reversal dead band and elbow hysteresis.** Slow single-joint moves settle within 0.1-0.2° of
  the goal, but a reversal waits 0.3-0.4° (most joints) or 1.7° (elbow, which also sags about 1°
  under gravity) before following. The servos' own dead-zone registers are 1 count (0.09°); this
  is friction against P 32. Phase 3's "arrived" tolerance of 0.7° is too tight for the elbow.
- **Wrist camera settings changed** (contract 7): white balance fixed at 3500 K (auto turned the
  warm room orange-red) and exposure cut from 30 ms to 12 ms with gain 40, because at 30 ms frames
  lost a third of their sharpness even at slow arm speeds (3-15°/s) and two thirds on fast moves.
  The 10 ms exposure-to-stamp offset (phase0.md) was measured at 30 ms; re-measure it with V4L2.
- **ffmpeg's `mjpeg` input ignores arrival times** and stamps a fixed 25 fps, so the first H.264
  scene video ran 186 s for 158 s of recording. `image2pipe` keeps arrival times. The CSV beside
  each scene video is the source of truth for its frame times.
- **Timing with cameras, recording and the console all running:** 0 skipped cycles and 0 errors
  per session; a few cycles per run start 3-7 ms late, and the worst bus work reaches 12-15 ms. The
  exit criterion's 10-minute histogram is taken with all of them on.
- **Tried and removed: fine mode** (follower motion scaled to a third of the leader's, switched by
  a key or by opening the gripper fully). It helped but wasn't the problem; the grip torque was.
- **Contact the arm can't see.** Pressing the gripper into the base doesn't show on the console's
  raw load bars, which also rise on fast moves and under gravity. A free-motion load model
  (gravity and acceleration, fitted from logged telemetry) would show only unexplained load; it
  belongs with the Phase 3 checks, not the loop.
- **Recording footprint:** the MCAP is 95% wrist JPEGs (35 KB each at gain 40, about 4 GB an hour).

## Tests (written first)

- Grid on a `ReplayClock`: cycles land on the grid; an overrun skips exactly the next cycle and shows
  as a sequence gap; ticks fall on every 4th cycle.
- Stepping: four cycles move the goal in equal steps from one target to the next; a skipped cycle
  doesn't change where the next tick's target lands.
- Ticks reference frames: each `Tick` names the latest frame of each camera at its time, from a
  synthetic camera with known times.
- Action queue: `n_action_steps` of each chunk, in order; the next chunk is requested in time; a
  late chunk holds the last target; ensembling, when on, matches LeRobot's weights.
- Mux: one active source; a switch starts from the present pose; `Tick.source` changes on that tick.
- Safety: range, slew and stall each clip and report the right `Limit`; the most severe wins in
  `Tick.clipped`; e-stop turns torque off within one cycle; the health filter ignores single junk
  readings (temperature and voltage) and trips on real ones.
- One conformance suite for the `Arm` trait, passed by `ServoArm` on `SimulatedServos` and by the
  mock arm; one for the camera trait, passed by the synthetic, replay and V4L2 cameras.
- Teleop end to end on both, as today's tests, plus: the session it writes reads back with every
  tick's frames and clipping.
- On the Spark, with the real arms and cameras: a 10-minute run's tick-jitter histogram and overrun
  count, with both cameras streaming and a recording running; the V4L2 camera's frames carry
  kernel stamps before their receive times and no sequence gaps.

## Exit criteria

From the spec: unit tests on the action queue and the safety checks (joint limits, rate limits,
e-stop), and a tick-jitter histogram from a 10-minute run. Plus: teleop runs on the new loop on the
real arms and cameras and writes a session (MCAP with the wrist camera, scene video beside it) that
passes the contract 1 Python check (a 10 s recording with the real cameras and simulated arms
passes since 2026-11-24; ticks don't reference frames yet); 20 starts in a row with no
first-cycle error.

## Out of scope

The policy itself (Phase 2; Phase 1 feeds the action queue test chunks), skills, checks and the
sequencer (Phase 3), the LeRobot plugin and the exporter to LeRobot datasets (Phase 4), episode
tags and per-skill segments in recordings (Phase 4, with the exporter). The byte trace stays a
separate file; moving it into the session as an optional channel can wait until something needs
the two together.
