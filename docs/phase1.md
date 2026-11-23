# Phase 1 design: the control loop on fakes

Status: design, 2026-11-23. Implements Phase 1 of [part2-spec.md](part2-spec.md), on the contracts
of [phase0.md](phase0.md). Names follow the glossary in the [README](../README.md).

Phase 1 builds the loop that every later phase runs inside: one loop for teleop, scripts and
policies, at fixed rates, with safety in it and every decision recorded. It runs on a mock arm and
replayed or synthetic cameras, so it needs no hardware, and on the real arms unchanged.

## The loops

A robot is nested loops, each at its own rate. Each one hands a setpoint down to the faster loop
below it and gets observations back. The code follows the same layers.

| Loop | Rate | Decides | Hands down | Gets back | Code |
|---|---|---|---|---|---|
| Task | per Hanoi move, ~0.05 Hz | which disc goes where next | a step | step results | `runtime/` (Phase 3) |
| Step | 2-10 s | run a skill until its check says done or failed | a policy or script | check results | skills, checks (Phase 3) |
| Chunk | each inference, ~1-3 s | the next stretch of motion | a `Chunk` of poses | an `Obs` | `policy/` (Phase 2) and the action queue |
| **Tick** | **30 Hz** (the camera) | this tick's target, within safety | a target `Pose` | `JointState`, latest `Frame` | `control/` |
| **Slot** | **120 Hz** (the servo grid) | the goal to write now, stall holds, slew limit | goal `Counts` | `Counts`, faults | `control/` and `arm/` |
| Servo firmware | ~kHz, in each STS3215 | motor current | — | encoder | not ours; only its gains (contract 8) |

Phase 1 builds the two loops in bold, and the action queue at the boundary with the chunk loop.

**Rules.**
1. **A slower loop never blocks a faster one.** Inference runs on its own thread and hands chunks
   to the action queue; the tick takes whatever is queued and never waits. Checks on frames
   (Phase 3) work the same way.
2. **Each loop talks only to its neighbours.** The tick never sees servos (the `Arm` trait is the
   boundary, enforced by `just check`); the sequencer never sees ticks.
3. **Safety lives at the fastest loop that can see the danger.** Stall holds and the slew limit
   need counts and faults on every write, so they run per slot. Joint ranges are clipped per tick.
   A wrong plan is caught by checks at the step loop.
4. **Setpoints go down and observations come up, in glossary types:** step → `Chunk` → target
   `Pose` → goal `Counts`; `Counts` → `JointState` → `Tick` and `Obs` → check result.

## Decisions

- **One thread runs both slots and ticks.** The grid runs at 120 Hz; every 4th slot is also a tick
  (120 / 30). One thread keeps the timing deterministic, needs no locks, and lets every timing test
  run on a replayed `Clock`. Two threads would only earn their keep if tick work overran a slot;
  inference is already off-thread.
- **Slots step linearly toward the tick's target.** A tick sets a target; the four slots up to the
  next tick move the goal in equal steps from the previous target to the new one. Smoother than
  holding the target for four slots (LeRobot's behaviour), and it means the teleop lead is
  extrapolated 30 times a second and smoothed, not 120 times a second on noisy velocity: the
  measured cause of target jitter (phase0.md).
- **Ensembling, when used, lives in the action queue at the tick**, the only place that sees
  overlapping chunks. The default stays the spec's: play `n_action_steps` of each chunk, ensembling
  off, as `lerobot-eval` does.
- **`Tick.clipped` covers its four slots:** a joint is listed if any of its slots was clipped,
  with the most severe reason (`Stall`, then `Step`, then `Range`).

## What gets built

All in `crates/so101/src/control/` unless noted. Every part works through the `Arm` trait, so it
runs on `ServoArm` and the mock arm alike.

**The grid, on `Clock`.** `Grid` takes its time from the `Clock` port instead of `Instant`, and
follows contract 5: slot *n* starts at `t0 + n × period`; an overrun skips the next slot and the
loop resumes on the grid. Slot index is `JointState`'s sequence, so a skipped slot is a gap in the
log. On a `ReplayClock` the timing tests become exact, which retires the two flaky tests (the
grid's wake-up test and teleop's tremor test both assert real OS scheduling today).

**The slot step** (120 Hz): read the follower's `JointState` and faults; update the stall watch
(teleop's logic today, unchanged); compute this slot's goal by stepping toward the tick's target;
apply the slew limit and any hold; write the goal; on one slot in four, read one actuator's
`ActuatorStatus`, rotating. Publishes `JointState`, the follower's `JointTarget`, and
`ActuatorStatus` when read.

**The tick** (30 Hz, every 4th slot): take the latest `JointState` and the latest `Frame` from each
camera; ask the active source for a target; clip it to the joint ranges; hand it to the next four
slots; publish a `Tick` with what was requested, what was sent and why they differ.

**Sources**, one interface: given the tick's observation, return a target `Pose` or nothing.
- **Teleop:** reads the leader `Arm` and returns its pose plus the lead. The lead's velocity is
  filtered (it multiplied target jitter 1.2-2.1× when computed per slot), and the lead belongs here,
  before clipping, never after (contract 2).
- **Script:** a list of waypoints, stored as targets (what was commanded), each with a duration.
- **Action queue:** turns `Chunk`s into one target per tick. Asks for the next chunk early enough
  that inference finishes before the current one runs out; holds the last target if it hasn't.
  Optional ensembling.

**The mux:** exactly one source is active. Switching is explicit (`Tick.source` changes on that
tick), and a newly active source starts from the follower's present pose, so nothing jumps.
Takeover from the leader (Phase 4) is a mux switch.

**Safety, by loop:**

| Check | Loop | Action |
|---|---|---|
| Joint range | tick | clip the target (`Limit::Range`) |
| Slew limit | slot | limit each goal's step from the last (`Limit::Step`) |
| Stall or overload | slot | hold at the contact point; the gripper keeps a small squeeze (`Limit::Stall`) |
| Actuator health | slot | act on the median of each joint's last 5 temperature and voltage readings (single readings are sometimes junk); count impossible values (contract 8) |
| E-stop | slot | torque off on the next slot; the session records why |
| Follower failing | slot | after 0.5 s of failed slots, torque off and stop (today's fail-safe) |

**Shake measure.** Per joint, the follower's motion in the 5-30 Hz band against its target's in
the same band, from the 120 Hz `JointState` stream. Follower moving more than it was told to is
shake. Logged per tick window as a health number; no action in Phase 1.

**The mock arm** (`arm/`, implements `Arm`): follows goals the way the measurements say the
follower does: about 50 ms of dead time (more under gravity), then a first-order approach with a
speed limit. Fault injection: stall at a position, overload flag, a joint that stops answering.
Fast and deterministic on a replayed clock; `--sim` can switch to it. `SimulatedServos` stays for
testing the servo driver byte for byte.

**Cameras on fakes:** a synthetic camera (generated frames with exact times) and a replay camera
(frames from a session or a LeRobot dataset video, at their recorded times). The camera trait
arrives with them, in `camera/`.

**The session, in place of Part 1's prints:** teleop publishes to the `EventBus`, the recorder
writes the session, and the header is built in one place, with the calibration files and hashes
(contract 6), each camera's device and read-back settings (contract 7), and the gains read back
from every servo (contract 8). The gains come from one file in `data/`, applied on every start.

**Smaller changes:** `JointState.t` is the midpoint of request and reply, not the reply; teleop
defaults to 120 Hz and the gripper to LeRobot's 50% torque.

## Tests (written first)

- Grid on a `ReplayClock`: slots land on the grid; an overrun skips exactly the next slot and shows
  as a sequence gap; ticks fall on every 4th slot.
- Stepping: four slots move the goal in equal steps from one target to the next; a skipped slot
  doesn't change where the next tick's target lands.
- Action queue: `n_action_steps` of each chunk, in order; the next chunk is requested in time; a
  late chunk holds the last target; ensembling, when on, matches LeRobot's weights.
- Mux: one active source; a switch starts from the present pose; `Tick.source` changes on that tick.
- Safety: range, slew and stall each clip and report the right `Limit`; the most severe wins in
  `Tick.clipped`; e-stop turns torque off within one slot; the health filter ignores single junk
  readings and trips on real ones.
- One conformance suite for the `Arm` trait, passed by `ServoArm` on `SimulatedServos` and by the
  mock arm.
- Teleop end to end on both, as today's tests, plus: the session it writes reads back with every
  tick's frames and clipping.
- On the Spark, with the real arms: a 10-minute run's tick-jitter histogram and overrun count.

## Exit criteria

From the spec: unit tests on the action queue and the safety checks (joint limits, rate limits,
e-stop), and a tick-jitter histogram from a 10-minute run. Plus: teleop runs on the new loop on the
real arms and writes a session that passes the contract 1 Python check; the flaky timing tests are
gone.

## Out of scope

Real cameras (Phase 4), the policy itself (Phase 2; Phase 1 feeds the action queue test chunks),
skills, checks and the sequencer (Phase 3), the LeRobot plugin (Phase 4). The byte trace stays a
separate file; moving it into the session as an optional channel can wait until something needs
the two together.
