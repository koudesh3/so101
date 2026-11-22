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
