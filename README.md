# so101

A from-scratch Rust driver and teleoperation loop for the LeRobot SO-101 leader and follower arms,
written to learn how they work.

## Layout

All code is under `src/`, split by what it does rather than by language:

```
src/
  proto/           the messages every part speaks, in protobuf (so101/v1), shared by every
                   language; Foxglove's image message; the frozen schema snapshot
  robot/           runs the arm (Rust). Each crate depends only on the ones above it:
    contracts/       the Rust side of the messages (generated from src/proto), the monotonic
                     clock, and the traits: ArmDriver, FrameSource, TargetSource, Policy
    session-log/     the event bus and the session log (MCAP): recorder, reader with drop
                     detection, replay, recovery
    drivers/         hardware drivers; today the SO-101 arm (arm::so101): servo protocol frames, the
                     STS3215 control table, the serial transport and byte log, one servo bus (real
                     or simulated), the six joints, calibration, an arm
    control/         the fixed-rate ticker, the calibration sweep, the teleop loop
    cli/             the `so101` command line, and the end-to-end tests against the simulated servos
  web/             what a person looks at in the browser; no build step
    dashboard/       the live dashboard and its replay mode
    viewer/          one arm, or the servo alone, in 3D
    assets/          the arm and servo meshes, built by tools/ from the SO-ARM100 model
    server/          the Rust backend: serves these apps (embedded in the binary), streams live
                     teleop events, replays a byte log as events, renders a byte log as an HTML report
  ml/              the LeRobot side
    act              record, teleop, calibrate and train with LeRobot on the Spark, in tmux
data/            what isn't code
  calibration/       the leader's and follower's calibration files (in git)
  robot.conf         the Spark's arm ports and camera settings, read by `act`
  logs/              recorded byte logs from teleop runs (not in git)
docs/
  part2-spec.md      the plan for Part 2
  devlog/            dated entries: what was tried, how, and what it showed
Cargo.toml       one Cargo workspace over src/robot and src/web/server
justfile         the commands for the whole repo; mise.toml pins the tools they need
```

## Where it's going

Part 2 ([the spec](docs/part2-spec.md)) turns this into a single-arm robot that solves 4-disc Tower of Hanoi: ACT policies for
grasp and place, scripted transits between pegs, and a sequencer over a catalog of skills. New
parts arrive with the phase that needs them, not before:

- Phase 0 is built: `contracts` and `log` ([the design](docs/phase0.md)). Skill, check and
  sequencer traits, and their messages, arrive in Phase 3
- `drivers` grows mock and replay arms, the leader as a target source, and USB cameras (Linux only)
- `control` grows the safety wrapper, the target mux and the action queue
- `src/robot/runtime`: skills, checks, the step runner, and the Hanoi sequencer
- `src/robot/policy-onnx`: the trained ACT policy run with ONNX Runtime; the only crate that
  needs a GPU
- `src/robot/so101-py`: PyO3 bindings, so LeRobot can record and train through this driver
- `src/ml/` grows training, the LeRobot plugin, ONNX export, checkpoint probes and parity tests

The calibration moves to LeRobot's file format and units (degrees, gripper 0-100) at the same time.

## Use

With [mise](https://mise.jdx.dev) installed, `mise install` fetches Rust, just and uv. Then:

```
just              # list the commands
just check        # formatting, clippy, the tests (simulated servos), and Python reading a log
just sim          # teleop the simulated arms, no hardware
```

With hardware (`so101` is the binary `cargo install --path src/robot/cli` puts on your path),
calibrate each arm, then teleoperate; `--serve` opens the dashboard at http://127.0.0.1:7878:

```
so101 calibrate --port <leader port> --arm-id leader --out leader.json
so101 calibrate --port <follower port> --arm-id follower --out follower.json
so101 --log run.jsonl teleop --leader <port> --follower <port> \
  --leader-calibration leader.json --follower-calibration follower.json --serve
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
- `just act <command>` runs `src/ml/act` there (`just act help` lists the commands). On the Spark,
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
