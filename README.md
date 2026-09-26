# so101

A from-scratch Rust driver for the LeRobot SO-101 arm pair: ping, scan,
monitor, calibrate, and 1:1 leader-follower teleop over two USB serial
ports, with every byte on both buses logged and a tool that turns the log
back into annotated frames and charts.

No LeRobot, no Python, no vendor SDK. The only runtime dependency that
touches hardware is the `serialport` crate.

## Layout

| Layer / module           | What it is                                                  | Tested by |
|--------------------------|-------------------------------------------------------------|-----------|
| `wire/protocol`          | frame encode/parse, checksum, sign-magnitude                | unit tests against the vendor manual's byte strings |
| `wire/registers`         | the STS3215 control table as data                           | unit tests |
| `wire/transport`         | `Transport` trait, serial port, `Recorder` byte logger      | integration tests via the simulator |
| `device/bus`             | ping / identify / read / write / sync read / sync write     | integration tests |
| `device/sim`             | simulated servos that answer real protocol bytes            | integration tests |
| `robot/joint`            | the six joints, their servo IDs and units                   | unit tests |
| `robot/calibration`      | ticks <-> degrees or percent; the calibration file          | unit tests |
| `robot/arm`              | six joints on one bus; the last layer that names a register | integration tests |
| `control/ticker`         | fixed-rate deadline scheduling                              | unit tests |
| `control/calibrate`      | the range sweep                                             | integration tests |
| `control/teleop`         | the loop                                                    | integration test: leader waves, follower follows, log decodes |
| `report/visualize`       | byte log -> frames -> HTML report                           | integration test |
| `report/dashboard`       | live dashboard: std-only HTTP + server-sent events, byte tap | integration test |
| `error`                  | the one error type everything above `wire` returns          | |

## Build and test

```
cargo build --release
cargo test
```

Every command takes `--sim` (simulated servos, no hardware) and
`--log FILE` (record every byte in both directions to JSON lines).

## Commands

```
so101 --sim ping --id 1
so101 --sim scan
so101 --sim monitor --hz 5
so101 --sim calibrate --arm-id sim --out sim.json          # press Enter twice
so101 --sim --log run.jsonl teleop --hz 30 --ticks 300
so101 visualize --log-file run.jsonl --out run.html
```

With hardware, replace `--sim` with ports:

```
so101 ping --port /dev/ttyACM0 --id 1
so101 scan --port /dev/ttyACM0
so101 calibrate --port /dev/ttyACM0 --arm-id leader   --out leader.json
so101 calibrate --port /dev/ttyACM1 --arm-id follower --out follower.json
so101 --log run.jsonl teleop --leader /dev/ttyACM0 --follower /dev/ttyACM1 \
      --leader-calibration leader.json --follower-calibration follower.json --hz 30
so101 visualize --log-file run.jsonl --out run.html
```

## Live dashboard

```
so101 --sim teleop --serve                     # simulated pick-and-place, open http://127.0.0.1:7878/
so101 teleop --leader $SO_LEADER --follower $SO_FOLLOWER \
      --leader-calibration leader.json --follower-calibration follower.json --max-step 50 --serve
```

`--serve [ADDR]` (default `127.0.0.1:7878`) streams every tick and every byte
on both buses to one page (`report/dashboard`), and turns on a telemetry side
channel: each tick also reads one follower servo's Present_Position..
Present_Current block in one round trip, rotating through the six, so load,
current, temperature, voltage and status refresh at 5 Hz per joint at 30 Hz.

The page's only control is Start/Stop (or Space); the arms drive the rest:

- **Workstation**: both arms from TheRobotStudio's meshes, posed through their
  URDFs, cel-shaded with colour laid down as ordered dither, heavy ink
  silhouette, thin inner contours. A servo is grey at rest and gains chroma
  with speed. The grey skeleton is the commanded pose when it differs.
- **Joint rows**: value, leader-follower gap, calibrated range rail, an
  auto-scaled 8 s trace, and load per read.
- **Joint space**: a fixed map. The six normalised joints are projected onto
  the three directions you move in most, learned from the first 20 s of
  motion after Start, then locked (axes, centre, scale), so a pose always
  lands in the same place. Saved in the browser while the calibration is
  unchanged; `?relearn` discards it. Isometric from a fixed cube corner, the
  follower drawn as a tapering tail coloured by the joints driving it.
- **Wire**: every request and reply, bus occupancy at 1 Mbaud, loop work
  against the period, the instructions on the wire decoded.
- **Attention**: stalls, hard-stop proximity, steady offsets, lag, speed
  limiting, heating, overload bits, bus errors, each with what to do and a
  ring on the servo in 3D.

Follows the system light/dark setting (`?theme=` overrides); every label
clears 4.5:1 and every chart mark 3:1 against the paper in both themes. If a
rendered joint turns opposite to the real one, flip its `SIGN` entry in
`src/report/dashboard.html`. `SO101_DASHBOARD=src/report/dashboard.html`
serves the page from disk so it can be edited without rebuilding. Demo flags:
`?synthetic` (scripted trajectories), `?virtual` (time advances only through
`__step(ms)`), `?solo=phase` (joint-space panel only).

The meshes in `src/report/assets/` are packed by `tools/build_model.py` from
[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) `Simulation/SO101`
(Apache-2.0, license in `src/report/assets/LICENSE.SO-ARM100`).

## Bring-up order on real hardware

1. `ping --id 1` on each port. Proves port, baud, wiring, power, jumper.
   It also reads the model number and expects 777.
2. `scan`. Expect exactly `[1, 2, 3, 4, 5, 6]`. Duplicates or gaps mean
   the ID assignment is wrong; fix that before anything else.
3. `monitor`. Turn joints by hand, watch ticks change, check voltage and
   temperature read sensibly.
4. `calibrate` each arm. Step one writes the homing offset via
   Torque_Enable=128; step two records the range you sweep.
5. `teleop` with `--ticks 100` first and a small `--max-step` (say 50) so
   a wrong calibration cannot lunge. Then remove the tick limit.
6. `visualize` the log. The dashed goal trace should lead the solid
   present-position trace by a small, steady amount.

## Byte log format

One JSON object per line:

```
{"t_us":1234,"port":"follower","dir":"tx","hex":"FFFFFE16832A02010008..."}
```

`t_us` is microseconds from a monotonic clock shared by both ports. `dir`
is `tx` (host to bus) or `rx` (bus to host). Frames are logged as the
transport saw them; the visualizer re-parses them and does not trust
record boundaries.

## What the simulator models, and what it does not

Models: the control table, response level, Lock/EEPROM, Torque_Enable=128
recentring, the homing offset in reported position, a velocity-limited
P-loop at 3100 ticks/s, sync read reply ordering, and silent servos.

Does not model: USB latency, the half-duplex direction switch, transmit
echo, checksum corruption on the wire, temperature, or load. `work` times
printed by `--sim teleop` are therefore not representative; expect a few
milliseconds per tick on real ports.

## Things the real hardware will decide

- Whether a plain WRITE gets a reply. `bus.write` treats a clean timeout
  as success so either Response_Level works.
- Whether the adapter echoes transmitted bytes. `bus` discards the input
  buffer before each request; if a ping with no servo attached returns
  6 bytes, add echo-skipping in `SerialTransport`.
- The real per-tick work time, which sets how high `--hz` can go.
