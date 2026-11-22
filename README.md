# so101

A from-scratch Rust driver and teleoperation loop for the LeRobot SO-101 leader and follower arms,
written to learn how they work.

## Layout

Split by what each part does, not by language:

```
robot/           runs the arm. Each crate depends only on the ones above it:
  drivers/           hardware drivers; today the SO-101 arm (arm::so101): servo protocol frames, the
                     STS3215 control table, the serial transport and byte log, one servo bus (real
                     or simulated), the six joints, calibration, an arm
  control/           the fixed-rate ticker, the calibration sweep, the teleop loop
  cli/               the `so101` command line, and the end-to-end tests against the simulated servos
ui/              what a person looks at in the browser; no build step
  dashboard/         the live dashboard and its replay mode
  viewer/            one arm, or the servo alone, in 3D
  assets/            the arm and servo meshes, built by tools/ from the SO-ARM100 model
  server/            the Rust backend: serves these apps (embedded in the binary), streams live
                     teleop events, replays a byte log as events, renders a byte log as an HTML report
data/            what isn't code
  calibration/       the leader's and follower's calibration files (in git)
  robot.conf         the Spark's arm ports and camera settings (the Spark reads it from its checkout)
  logs/              recorded byte logs from teleop runs (not in git)
ops/             running the Spark, where the arms and cameras are plugged in: the `act` wrapper
                 around LeRobot, and the camera characterisation (see its README)
docs/            part2-spec.md, the plan for Part 2
Cargo.toml       one Cargo workspace over robot/ and ui/server
justfile         the commands for the whole repo; mise.toml pins the tools they need
```

## Where it's going

Part 2 ([the spec](docs/part2-spec.md)) turns this into a single-arm robot that solves 4-disc Tower of Hanoi: ACT policies for
grasp and place, scripted transits between pegs, and a sequencer over a catalog of skills. New
parts arrive with the phase that needs them, not before:

- `robot/contracts`: the message types (JointState, JointTarget, Frame, Obs, Chunk, ...),
  the clock, and the traits every module implements (ArmDriver, FrameSource, Policy, Skill,
  Check, Sequencer)
- `robot/log`: the event bus, recorder and reader; everything is logged and replayable
- `drivers` grows mock and replay arms, the leader as a target source, and USB cameras (Linux only)
- `control` grows the safety wrapper, the target mux and the action queue
- `robot/runtime`: skills, checks, the step runner, and the Hanoi sequencer
- `robot/policy-onnx`: the trained ACT policy run with ONNX Runtime; the only crate that
  needs a GPU
- `robot/so101-py`: PyO3 bindings, so LeRobot can record and train through this driver
- `ml/`: training with LeRobot, the LeRobot plugin, ONNX export, checkpoint probes and parity tests

The calibration moves to LeRobot's file format and units (degrees, gripper 0-100) at the same time.

## Use

With [mise](https://mise.jdx.dev) installed, `mise install` fetches Rust, just and uv. Then:

```
just              # list the commands
just check        # clippy and the tests, against simulated servos
just sim          # teleop the simulated arms, no hardware
```

With hardware (`so101` is the binary `cargo install --path robot/cli` puts on your path),
calibrate each arm, then teleoperate; `--serve` opens the dashboard at http://127.0.0.1:7878:

```
so101 calibrate --port <leader port> --arm-id leader --out leader.json
so101 calibrate --port <follower port> --arm-id follower --out follower.json
so101 --log run.jsonl teleop --leader <port> --follower <port> \
  --leader-calibration leader.json --follower-calibration follower.json --serve
```

The arms and cameras are plugged into the Spark: `just spark <recipe>` runs any of these there
(see [ops/README.md](ops/README.md)).

`just teleop` runs the last of these with the files in `data/calibration/`, logging to `data/logs/`, taking the ports from
`LEADER` and `FOLLOWER`.

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
