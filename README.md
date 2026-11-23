# so101

A from-scratch Rust driver and teleoperation loop for the LeRobot SO-101 leader and follower arms,
written to learn how they work.

## Layout

```
crates/so101/    the Rust code, one crate, one folder per thing (src/):
  arm/             the arm: joints, poses, calibration, `Arm`, and servo/ (the STS3215 bus underneath);
                   Part 1's teleop loop until Phase 1 moves it into control/
  camera/          frames (the V4L2 code joins it in Phase 4)
  policy/          observations and action chunks (the ONNX runner joins it in Phase 2)
  control/         the loop: the grid of slots, ticks, events, the event bus
  log/             the session log: protobuf in MCAP, recorder, reader, recovery
  clock.rs         timestamps, real and replayed time
  bin/so101.rs     the `so101` command line, which puts the pieces together
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
  devlog/          dated entries: what was tried, how, and what it showed
justfile         the commands for the whole repo; mise.toml pins the tools they need
```

## Glossary

One concept, one name, one home.

| Name | Means | Lives in |
|---|---|---|
| `Joint` | one of the six, in LeRobot's order | `arm` |
| `Pose` | where all six are: degrees, gripper 0-100 | `arm` |
| `Counts` | raw encoder readings, one per joint | `arm` |
| `Calibration` | maps `Counts` to a `Pose`, and how to make one | `arm` |
| `Arm` | the physical arm: read its pose, send a target, read its actuators | `arm` |
| `ServoBus` | the STS3215 wire protocol underneath `Arm` | `arm/servo` |
| `JointState` / `JointTarget` | a pose read / a pose commanded, with time and counts | `arm` |
| `ActuatorStatus` | one joint's temperature, current, load, faults | `arm` |
| `Frame` | one camera image with its times | `camera` |
| `Chunk` | a policy's predicted poses | `policy` |
| `Grid` / `Slot` | the servo loop's fixed schedule / one step of it | `control` |
| `Tick` | what the loop saw and sent at one control step | `control` |
| `Source` | who produced a target: teleop, script, policy | `control` |
| `Event` | anything published on the `EventBus`, with its times; it carries a `Message` | `control` |
| `Log` | the session recording | `log` |
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
so101 --log run.jsonl teleop --leader <port> --follower <port> \
  --leader-calibration leader.json --follower-calibration follower.json
```

`just teleop` runs the last of these with the files in `data/calibration/`, logging to `data/logs/`, taking the ports from
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
