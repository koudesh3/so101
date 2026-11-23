# Run `just` to list recipes.

default:
    @just --list

# Formatting, the layer rule, clippy (warnings fail), and the tests (simulated servos, no hardware; one runs Python on a log)
check:
    cargo fmt --check
    @just check-layers
    cargo clippy --workspace --all-targets -- -D warnings
    cargo test --workspace

# Each module uses only the ones below it (clock, camera, arm, policy, control, session), and the
# control loop reaches arms only through the Arm trait
check-layers:
    #!/usr/bin/env bash
    set -euo pipefail
    cd crates/so101/src
    order=(clock camera arm policy control session)
    for i in "${!order[@]}"; do
        module=${order[$i]}
        path=$module; [ -d "$module" ] || path=$module.rs
        for above in "${order[@]:$((i + 1))}"; do
            if grep -rnE "crate::$above\b" "$path"; then
                echo "error: $module uses $above, which sits above it (order: ${order[*]})"; exit 1
            fi
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

# Push this commit to the Spark and run a recipe there, e.g. `just spark check` (only committed work goes)
spark *args:
    @git diff --quiet HEAD || echo "note: uncommitted changes stay on this machine"
    git push -q spark HEAD:main
    ssh -t spark 'cd /projects/so101 && ~/.local/bin/mise exec -- just {{args}}'

# Run the LeRobot wrapper on the Spark, e.g. `just act devices` (see README.md)
act *args:
    ssh -t spark act {{args}}
