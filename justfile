# Run `just` to list recipes.

default:
    @just --list

# Formatting, the core/adapter rule, clippy (warnings fail), the tests (simulated servos, no hardware), and the Python log check
check:
    cargo fmt --check
    @just check-layers
    cargo clippy --workspace --all-targets -- -D warnings
    cargo test --workspace
    @just check-py

# The control loop uses the arm's types, never its hardware or the log; only bin/ wires those in
check-layers:
    #!/usr/bin/env bash
    set -euo pipefail
    cd crates/so101/src
    if grep -rnE 'crate::(log|arm::(Arm|servo|teleop|calibration))\b' control/; then
        echo "error: control/ reaches into hardware or the log (above)"; exit 1
    fi

# Python reads a session log with no generated code (docs/phase0.md, contract 1)
check-py:
    #!/usr/bin/env bash
    set -euo pipefail
    d=$(mktemp -d "${TMPDIR:-/tmp}/so101.XXXXXX"); trap 'rm -rf "$d"' EXIT
    cargo run -q -p so101 --example write_sample -- "$d/sample.mcap" >/dev/null
    uv run -q crates/so101/tests/read_log.py "$d/sample.mcap"

# Refresh the so101.v1 schema snapshot after an allowed (additive) .proto change
proto-snapshot:
    SO101_UPDATE_PROTO_SNAPSHOT=1 cargo test -q -p so101 --test schema_snapshot

# Format the Rust code
fmt:
    cargo fmt

# Build the so101 binary (release)
build:
    cargo build --release

# Teleop against the simulated arms, logging to a new timestamped file in data/logs/
sim ticks="300":
    cargo run --release -- --sim --log data/logs/sim-$(date +%Y%m%d-%H%M%S).jsonl teleop --ticks {{ticks}}

# Teleop the real arms (set LEADER and FOLLOWER to the ports); logs to a new timestamped file in data/logs/
teleop:
    cargo run --release -- --log data/logs/teleop-$(date +%Y%m%d-%H%M%S).jsonl teleop \
      --leader "$LEADER" --follower "$FOLLOWER" \
      --leader-calibration data/calibration/leader.json --follower-calibration data/calibration/follower.json

# Push this commit to the Spark and run a recipe there, e.g. `just spark check` (only committed work goes)
spark *args:
    @git diff --quiet HEAD || echo "note: uncommitted changes stay on this machine"
    git push -q spark HEAD:main
    ssh -t spark 'cd /projects/so101 && ~/.local/bin/mise exec -- just {{args}}'

# Run the LeRobot wrapper on the Spark, e.g. `just act devices` (see README.md)
act *args:
    ssh -t spark act {{args}}
