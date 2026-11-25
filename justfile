# Run `just` to list recipes.

default:
    @just --list

# Formatting, the layer rule, clippy (warnings fail), and the tests (simulated servos, no hardware; one runs Python on a log)
check:
    cargo fmt --check
    @just check-layers
    cargo clippy --workspace --all-targets -- -D warnings
    cargo test --workspace

# Each module uses only modules on lower levels (clock; arm and camera; policy; control; session),
# never one beside it; and the control loop reaches arms only through the Arm trait
check-layers:
    #!/usr/bin/env bash
    set -euo pipefail
    cd crates/so101/src
    levels=("clock" "arm camera" "policy" "control" "session")
    for i in "${!levels[@]}"; do
        for module in ${levels[$i]}; do
            path=$module; [ -d "$module" ] || path=$module.rs
            for other in ${levels[@]:$i}; do
                [ "$other" = "$module" ] && continue
                if grep -rnE "crate::$other\b" "$path"; then
                    echo "error: $module uses $other, which isn't below it (levels: ${levels[*]})"; exit 1
                fi
            done
        done
    done
    if grep -rnE 'crate::session\b|crate::arm::(servos|uart|simulated_servos|calibration)\b|\bServoArm\b' control/; then
        echo "error: control/ reaches past the Arm trait (above)"; exit 1
    fi

# Refresh the so101.v1 schema snapshot after an allowed (additive) .proto change
proto-snapshot:
    SO101_UPDATE_PROTO_SNAPSHOT=1 cargo test -q -p so101 --test schema_snapshot

# Format the Rust code
fmt:
    cargo fmt

# Build the so101 binary (release)
build:
    cargo build --release

# Teleop against the simulated arms, tracing every byte to a new timestamped file in data/traces/
sim ticks="300":
    cargo run --release -- --sim --trace data/traces/sim-$(date +%Y%m%d-%H%M%S).jsonl teleop --ticks {{ticks}}

# Teleop the real arms (set LEADER and FOLLOWER to the ports); traces every byte to a new timestamped file in data/traces/
teleop:
    cargo run --release -- --trace data/traces/teleop-$(date +%Y%m%d-%H%M%S).jsonl teleop \
      --leader "$LEADER" --follower "$FOLLOWER" \
      --leader-calibration data/calibration/leader.json --follower-calibration data/calibration/follower.json

# On the Spark: teleop with both cameras and the console, ports and camera settings from
# /projects/robot.conf; traces every byte to data/traces/, recordings go to data/logs/
rig:
    #!/usr/bin/env bash
    set -euo pipefail
    . /projects/robot.conf
    controls=()
    for entry in "${CAMERA_CONTROLS[@]}"; do controls+=(--camera-controls "$entry"); done
    cargo run --release -q -- --trace data/traces/rig-$(date +%Y%m%d-%H%M%S).jsonl teleop --hz 120 \
      --leader "$LEADER_PORT" --follower "$FOLLOWER_PORT" \
      --leader-calibration data/calibration/leader.json --follower-calibration data/calibration/follower.json \
      --wrist-camera "$WRIST_CAMERA" --scene-camera "$SCENE_CAMERA" "${controls[@]}"

# Run a recipe or cargo command in Linux (arm64, like the Spark) on the laptop, e.g. `just linux check`
# or `just linux cargo test -p so101 --test cameras`: builds the Linux-only code (V4L2) without
# the Spark. Its own target dir and cargo cache, in Docker volumes.
linux *args:
    docker build -q -t so101-linux -f tools/linux.Dockerfile tools >/dev/null
    docker run --rm -t -v "$PWD":/src -w /src \
      -v so101-linux-target:/src/target -v so101-linux-cargo:/usr/local/cargo/registry \
      -v so101-linux-uv:/root/.cache/uv \
      so101-linux bash -c 'if [ "${1:-}" = cargo ]; then "$@"; else just "$@"; fi' _ {{args}}

# On the Spark: encode any recording's scene video still raw (teleop starts this itself when a
# recording stops; this finishes any it didn't, e.g. after a crash)
encode-scenes:
    cargo run --release -q -- encode-scenes data/logs

# On the laptop: tunnel the Spark's teleop console to http://localhost:8090 (Ctrl-C to close)
console host="spark":
    @echo "console: http://localhost:8090"
    ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 -L 8090:127.0.0.1:8090 {{host}}

# Push this commit to the Spark and run a recipe there, e.g. `just spark check` (only committed work goes)
spark *args:
    @git diff --quiet HEAD || echo "note: uncommitted changes stay on this machine"
    git push -q spark HEAD:main
    ssh -t spark 'cd /projects/so101 && ~/.local/bin/mise exec -- just {{args}}'

# Run the LeRobot wrapper on the Spark, e.g. `just act devices` (see README.md)
act *args:
    ssh -t spark act {{args}}
