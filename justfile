# Run `just` to list recipes.

default:
    @just --list

# Rust: clippy and the tests (simulated servos, no hardware)
check:
    cd robot && cargo clippy --workspace --all-targets
    cd robot && cargo test --workspace

# Build the so101 binary (release)
build:
    cd robot && cargo build --release

# Teleop against the simulated arms, logging to run.jsonl
sim ticks="300":
    cd robot && cargo run --release -- --sim --log ../run.jsonl teleop --ticks {{ticks}}

# Teleop the real arms with the dashboard at http://127.0.0.1:7878 (set LEADER and FOLLOWER to the ports)
teleop:
    cd robot && cargo run --release -- --log ../run.jsonl teleop \
      --leader "$LEADER" --follower "$FOLLOWER" \
      --leader-calibration ../calibration/leader.json --follower-calibration ../calibration/follower.json --serve

# Rebuild ui/assets from a checkout of TheRobotStudio's SO-ARM100 repo
meshes so_arm100:
    uv run ui/tools/build_model.py {{so_arm100}}/Simulation/SO101
    uv run ui/tools/build_servo.py {{so_arm100}}/Simulation/SO101
