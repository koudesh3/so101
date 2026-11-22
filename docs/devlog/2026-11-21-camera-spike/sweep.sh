#!/bin/bash
# per-mode sweep, 8 s each. probe args: dev W H FPS SECS FOURCC
cd "$(dirname "$0")"; DEV=/dev/video0
for m in "640 360 30 MJPG" "640 480 30 MJPG" "800 600 30 MJPG" "1024 576 30 MJPG" "1280 720 30 MJPG" "1280 960 30 MJPG" "1920 1080 30 MJPG" \
         "640 360 30 YUYV" "640 480 30 YUYV" "1280 720 10 YUYV" "1280 720 5 YUYV" "1920 1080 5 YUYV"; do
  set -- $m; echo "-- $m"; ./v4l2probe $DEV $1 $2 $3 8 $4 2>&1 | grep -v "^  gap"
done
