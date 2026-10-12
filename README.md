# so101

A from-scratch Rust driver and teleoperation loop for the LeRobot SO-101 leader and follower arms,
written to learn how they work. The post that goes with it:
[Teleoperation from Zero](https://nofreelun.ch/posts/devlog-1.html).

## Layout

```
src/
  wire/          servo protocol frames, the STS3215 control table, the serial transport and byte log
  device/        one servo bus: ping, read, write, sync read, sync write; real or simulated
  robot/         the six joints, calibration, an arm
  control/       the fixed-rate ticker, the calibration sweep, the teleop loop
  report/        the live dashboard (and its replay mode, and a one-arm 3D viewer); byte log -> HTML report
  main.rs        the `so101` command line
tests/           end-to-end tests against the simulated servos
calibration/     the leader's and follower's calibration files
episodes/        grab1, the recorded episode the post's figures are drawn from (a gzipped byte log)
figures/         scripts that draw the post's figures (see its README)
tools/           build the dashboard's arm and servo meshes from the SO-ARM100 model
```

## Use

```
cargo test
cargo run --release -- --sim --log run.jsonl teleop --ticks 300     # simulated arms, no hardware
cargo run --release -- --help
```

With hardware (`so101` is the binary `cargo install --path .` puts on your path), calibrate each
arm, then teleoperate; `--serve` opens the dashboard at http://127.0.0.1:7878:

```
so101 calibrate --port <leader port> --arm-id leader --out leader.json
so101 calibrate --port <follower port> --arm-id follower --out follower.json
so101 --log run.jsonl teleop --leader <port> --follower <port> \
  --leader-calibration leader.json --follower-calibration follower.json --serve
```

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
