# Run `just` to list recipes.

default:
    @just --list

# Rust: clippy and the tests (simulated servos, no hardware)
check:
    cargo clippy --workspace --all-targets
    cargo test --workspace

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

# Rebuild ui/assets from a checkout of TheRobotStudio's SO-ARM100 repo
meshes so_arm100:
    uv run ui/tools/build_model.py {{so_arm100}}/Simulation/SO101
    uv run ui/tools/build_servo.py {{so_arm100}}/Simulation/SO101
