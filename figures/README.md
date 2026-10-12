# Figures

The scripts that draw the post's figures from this repo's data. Standard library Python unless
noted; each writes to stdout or the path it is given.

| Script | Figure |
|---|---|
| `tick_hist.py` | Bus time per tick, with and without the dashboard's telemetry read (`--narrow`: the phone version) |
| `wire_overview.py` | The teleop loop: leader, host and follower, and what each sends each tick (`--narrow`: stacked, for a phone) |
| `trajectory_pca.py` + `trajectory_template.html` | An episode's actions and observations as two paths through joint space (PCA) |
| `wiring/` | The cel-shaded wiring diagram, rendered in Blender (see its README; `compose.py` takes a text scale for the phone labels) |
| `synthetic_episode.py` | A scripted episode in `so101 replay`'s format, for trying `trajectory_pca.py` |
| `encoder_unwrap.py` | One servo turn unrolling into the encoder's 4096 positions (animated SVG) |
| `frame_write_one.py` | One servo's write packet, byte by byte |
| `joint_ranges.py` | How much of a full turn each joint can use, from the URDF |

From the repo root, with the episode in `episodes/`:

```
python3 figures/tick_hist.py episodes/grab1.jsonl.gz tick-time.svg

gunzip -c episodes/grab1.jsonl.gz > grab1.jsonl
cargo run --release -- replay --log-file grab1.jsonl \
  --leader-calibration calibration/leader.json --follower-calibration calibration/follower.json \
  --out grab1.events.jsonl
python3 figures/trajectory_pca.py grab1.events.jsonl > trajectory.html
```
