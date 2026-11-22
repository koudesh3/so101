# ops

Running the Spark (DGX Spark, Ubuntu aarch64), where the arms and cameras are plugged in. Driven
from the laptop over SSH (`Host spark`).

- `spark/act`: record, teleop, calibrate and train with LeRobot, in tmux. Installed on the Spark at
  `~/.local/bin/act`; it reads `/projects/robot.conf` (a copy is in `data/robot.conf`), keeps
  LeRobot at `/projects/lerobot`, datasets in `/projects/datasets` and runs in `/projects/runs`.
  `ssh spark act help` lists the commands.
- `camspike/`: the camera characterisation from 2026-11-21 (frame rate, drops, latency, exposure,
  control persistence), with the logs it produced. The C tools build with `gcc -O2`; compiled
  binaries, captured frames and images stay on the Spark in `~/camspike`.
  - `run.sh`, `sweep.sh`, `v4l2probe.c`: the wrist camera (Innomaker U20CAM), every mode, then 10-minute runs
  - `cam2/`: the scene camera (icSpring): modes, auto-exposure frame drops, the AE-priority probe
  - `both.sh`: both cameras streaming at once on one hub
  - `g2g/`: glass-to-glass latency capture
  - `persist/grab.sh`: whether camera controls survive restarts and replugs
