# Run `just` to list recipes.

default:
    @just --list

# Formatting, clippy (warnings fail), the tests (simulated servos, no hardware), and the Python log check
check:
    cargo fmt --check
    cargo clippy --workspace --all-targets -- -D warnings
    cargo test --workspace
    @just check-py

# Python reads a session log with no generated code (docs/phase0.md, contract 1)
check-py:
    #!/usr/bin/env bash
    set -euo pipefail
    d=$(mktemp -d); trap 'rm -rf "$d"' EXIT
    cargo run -q -p session-log --example write_sample -- "$d/sample.mcap" >/dev/null
    uv run -q src/robot/log/tests/read_log.py "$d/sample.mcap"

# Refresh the so101.v1 schema snapshot after an allowed (additive) .proto change
proto-snapshot:
    SO101_UPDATE_PROTO_SNAPSHOT=1 cargo test -q -p contracts --test schema_snapshot

# Format the Rust code
fmt:
    cargo fmt

# Build the so101 binary (release)
build:
    cargo build --release

# Teleop against the simulated arms, logging to a new timestamped file in data/logs/
sim ticks="300":
    cargo run --release -- --sim --log data/logs/sim-$(date +%Y%m%d-%H%M%S).jsonl teleop --ticks {{ticks}}

# Teleop the real arms with the dashboard at http://127.0.0.1:7878 (set LEADER and FOLLOWER to the ports); logs to a new timestamped file in data/logs/
teleop:
    cargo run --release -- --log data/logs/teleop-$(date +%Y%m%d-%H%M%S).jsonl teleop \
      --leader "$LEADER" --follower "$FOLLOWER" \
      --leader-calibration data/calibration/leader.json --follower-calibration data/calibration/follower.json --serve

# Rebuild src/web/assets from a checkout of TheRobotStudio's SO-ARM100 repo
meshes so_arm100:
    uv run src/web/tools/build_model.py {{so_arm100}}/Simulation/SO101
    uv run src/web/tools/build_servo.py {{so_arm100}}/Simulation/SO101

# Push this commit to the Spark and run a recipe there, e.g. `just spark check` (only committed work goes)
spark *args:
    @git diff --quiet HEAD || echo "note: uncommitted changes stay on this machine"
    git push -q spark HEAD:main
    ssh -t spark 'cd /projects/so101 && ~/.local/bin/mise exec -- just {{args}}'

# Run the LeRobot wrapper on the Spark, e.g. `just act devices` (see README.md)
act *args:
    ssh -t spark act {{args}}
