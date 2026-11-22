#!/bin/bash
cd "$(dirname "$0")"; D=/dev/video2
v4l2-ctl -d $D -c auto_exposure=1 -c exposure_time_absolute=300
echo "== controls during run: $(v4l2-ctl -d $D -C auto_exposure,exposure_time_absolute | tr "\n" " ")"
for m in "1920 1080" "640 480"; do set -- $m; echo "== long run 600s: MJPG $1x$2"; ../v4l2probe $D $1 $2 30 600 MJPG; done
v4l2-ctl -d $D -c auto_exposure=3; echo "== restored: $(v4l2-ctl -d $D -C auto_exposure)"
