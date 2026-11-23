# 2026-11-21: camera spike

Characterising both cameras on the Spark before any recording: frame rate, drops, latency,
exposure, and whether control settings stick. The findings are summarised in the spec's open
questions ([part2-spec.md](../../part2-spec.md)); the scripts and the logs they produced are here.
The C tools build with `gcc -O2`; compiled binaries, captured frames and images stay on the Spark
in `/projects/spikes/camspike`.

**Correction (2026-11-23):** this spike had the roles swapped. The Innomaker U20CAM is the scene
camera and the icSpring is the wrist camera; the notes below keep the original labels.

- `run.sh`, `sweep.sh`, `v4l2probe.c`: the wrist camera (Innomaker U20CAM), every mode, then
  10-minute runs; results in `spike.log` and `sweep.log`
- `cam2/`: the scene camera (icSpring): modes, auto-exposure frame drops, the AE-priority probe
- `both.sh`: both cameras streaming at once on one hub; results in `both.log`
- `g2g/`: glass-to-glass latency capture
- `persist/grab.sh`: whether camera controls survive restarts and replugs
