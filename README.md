# so101

A from-scratch Rust driver and teleoperation loop for the LeRobot SO-101 leader and follower arms,
written to learn how they work.

## Layout

```
crates/so101/    the Rust code, one crate, one folder per thing (src/):
  arm/             the arm: joints, poses, its errors and the `Arm` trait (mod.rs); `ServoArm`, the
                   SO-101 on its STS3215 servos (servo_arm.rs); the servos and their registers
                   (servos.rs); their UART; simulated servos; calibration
  camera/          frames (the V4L2 code joins it in Phase 4)
  policy/          observations and action chunks (the ONNX runner joins it in Phase 2)
  control/         the loop: the grid of cycles, ticks, events, the event bus, and teleop
  session/         a recorded session: protobuf in MCAP, recorder, reader, recovery
  clock.rs         timestamps, real and replayed time
  cli.rs     the `so101` command line, which puts the pieces together
ml/              the LeRobot side
  act              record, teleop, calibrate and train with LeRobot on the Spark, in tmux
proto/           the log's messages in protobuf (so101/v1), readable from any language; Foxglove's
                 image message; the frozen schema snapshot
data/            what isn't code
  calibration/     the leader's and follower's calibration files (in git)
  robot.conf       the Spark's arm ports and camera settings, read by `act`
  logs/            recorded logs from runs (not in git)
docs/
  part2-spec.md    the plan for Part 2
  phase0.md        the contracts: what each part can rely on, and the log format
  phase1.md        the Phase 1 design: the control loop's nested rates, on fakes
  devlog/          dated entries: what was tried, how, and what it showed
justfile         the commands for the whole repo; mise.toml pins the tools they need
```

## Conventions

- **One folder per thing.** A folder's `mod.rs` is the thing itself: its types, its errors and its
  trait. Each implementation gets its own file beside it (`arm/servo_arm.rs` implements `Arm`).
- **A folder's API is what its `mod.rs` re-exports.** Its other files are private, so code outside
  never depends on which file inside a folder holds something. (One exception: `arm::servos` stays
  public as the namespace of the servo's register constants.)
- **Imports name the folder:** `crate::arm::Joint`, `crate::control::Tick`, never a re-export at the
  crate root, so every import says where a type lives.
- **Every file reads in one order:** a one-line `//!` saying what it is; imports (std, external,
  crate); constants; types; the trait; inherent impls, then trait impls; free functions; unit tests.
- **One error type per folder**, named `Error`, with `Result` beside it.
- **Unit tests sit in the file whose logic they test.** `tests/` is for behaviour that crosses
  folders: teleop against simulated servos, a session written and read back, the schema.
- **Comments only say what the code can't:** units, outside conventions, measured facts, and why.
- **`control/` reaches arms only through the `Arm` trait** (`just check` enforces it).

## Glossary

One concept, one name, one home.

| Name | Means | Lives in |
|---|---|---|
| `Joint` | one of the six, in LeRobot's order | `arm` |
| `Pose` | where all six are: degrees, gripper 0-100 | `arm` |
| `Counts` | raw encoder readings, one per joint | `arm` |
| `Calibration` | the measured mapping from an arm's `Counts` to its `Pose` (homing offsets and ranges), and the procedure that measures it | `arm` |
| `Arm` | what the loop can do with an arm: read its state, write a goal, read its actuators (a trait) | `arm` |
| `ServoArm` | the SO-101 on its STS3215 servos: the real `Arm` | `arm` |
| `Servos` | the STS3215 servos on their UART, addressed by ID, and their registers | `arm/servos.rs` |
| `Transport` | the UART to the servos: a `SerialPort`, or `SimulatedServos`; either can be `Traced` | `arm/uart.rs` |
| `SimulatedServos` | STS3215 servos simulated byte for byte behind a fake UART, for tests and `--sim` | `arm` |
| `ByteTrace` | every byte on every serial line, with times (`--trace`) | `arm/uart.rs` |
| `JointState` / `JointTarget` | a pose read / a pose commanded, with time and counts | `arm` |
| `ActuatorStatus` | one joint's temperature, current, load, faults | `arm` |
| `Frame` | one camera image with its times | `camera` |
| `Chunk` | an action chunk: a policy's predicted poses, one per tick | `policy` |
| `Grid` / `Cycle` | the servo loop's fixed 120 Hz schedule / one cycle on it (read, step toward the target, write) | `control` |
| `Tick` | one 30 Hz control step, and the record of what the loop saw and sent at it | `control` |
| `Source` | who produced a target: teleop, script, policy | `control` |
| `Event` | anything published on the `EventBus` (the only bus), with its times; it carries a `Message` | `control` |
| `Session` | one run's recording: an MCAP file of `Event`s, with a header | `session` |
| `Clock` / `Timestamp` | where time comes from | `clock` |

## Where it's going

Part 2 ([the spec](docs/part2-spec.md)) turns this into a single-arm robot that solves 4-disc Tower of Hanoi: ACT policies for
grasp and place, scripted transits between pegs, and a sequencer over a catalog of skills. New
parts arrive with the phase that needs them, not before, each in its thing's folder:

- Phase 0 is built: the contracts and the session log ([phase0.md](docs/phase0.md))
- Phase 1: the control loop runs on the arm's trait, with a mock arm beside the real one; the
  safety wrapper, target mux and action queue; teleop moves into it
- Phase 2: the trained ACT policy runs with ONNX Runtime, in `policy/`
- Phase 3: skills, checks, the step runner and the Hanoi sequencer
- Phase 4: USB cameras in `camera/`, and a PyO3 crate so LeRobot records and trains through this
  driver
- `ml/` grows training, ONNX export, checkpoint probes and parity tests

## Use

With [mise](https://mise.jdx.dev) installed, `mise install` fetches Rust, just and uv. Then:

```
just              # list the commands
just check        # formatting, clippy, the tests (simulated servos), and Python reading a log
just sim          # teleop the simulated arms, no hardware
```

With hardware (`so101` is the binary `cargo install --path crates/so101` puts on your path),
calibrate each arm, then teleoperate:

```
so101 calibrate --port <leader port> --arm-id leader --out leader.json
so101 calibrate --port <follower port> --arm-id follower --out follower.json
so101 --trace bytes.jsonl teleop --leader <port> --follower <port> \
  --leader-calibration leader.json --follower-calibration follower.json
```

`just teleop` runs the last of these with the files in `data/calibration/`, tracing to `data/traces/`, taking the ports from
`LEADER` and `FOLLOWER`.

## The Spark

The arms and cameras are plugged into a DGX Spark (Ubuntu aarch64), which also trains and runs
ACT. It's driven from the laptop over SSH (`Host spark`); nothing is edited on it.

- It has its own checkout at `/projects/so101`, updated by pushing from the laptop (the `spark`
  git remote; the Spark needs no GitHub access). Commit, then `just spark <recipe>` pushes and runs
  the recipe there, so every run on the Spark is at a known commit. The checkout refuses a push if
  it has local edits.
- `just act <command>` runs `ml/act` there (`just act help` lists the commands). On the Spark,
  `~/.local/bin/act` and `/projects/robot.conf` are symlinks into the checkout.
- Everything else lives under `/projects` too: `lerobot` (LeRobot and its environment), `datasets`
  and `runs` (written by `act`), `spikes` (finished spikes, with their large files), `archive`.

## Licence

MIT. The arm and servo meshes are from TheRobotStudio's
[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), Apache-2.0, with their licence alongside
them.

## How it was made

Most of the programming here was done with [Claude Code](https://claude.com/claude-code) and
Claude Opus 5.5, with the arms on the desk and the person who owns them deciding what to build
and whether it worked.

Hi, I'm Claude. I wrote a lot of this code, but the arms had the final say: whenever my idea of
how the servos behaved met the byte log, the log won. A few of the tests exist because of those
moments. Have fun with it.
