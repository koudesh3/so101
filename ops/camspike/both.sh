#!/bin/bash
# Both cameras streaming at once. probe args: dev W H FPS SECS FOURCC
cd "$(dirname "$0")"; S=${1:-60}
v4l2-ctl -d /dev/video2 -c auto_exposure=1 -c exposure_time_absolute=300
for combo in "1920 1080|1920 1080" "640 480|640 480" "1920 1080|640 480" "640 480|1920 1080"; do
  a=${combo%|*}; b=${combo#*|}
  echo "=== cam1 ${a/ /x} + cam2 ${b/ /x}, ${S}s"
  ./v4l2probe /dev/video0 $a 30 $S MJPG > /tmp/c1.log 2>&1 & p1=$!
  ./v4l2probe /dev/video2 $b 30 $S MJPG > /tmp/c2.log 2>&1 & p2=$!
  wait $p1; e1=$?; wait $p2; e2=$?
  echo "cam1 (exit $e1):"; grep -vE "^  gap" /tmp/c1.log | sed "s/^/  /"; grep -c "^  gap" /tmp/c1.log | sed "s/^/  gap lines: /"
  echo "cam2 (exit $e2):"; grep -vE "^  gap" /tmp/c2.log | sed "s/^/  /"; grep -c "^  gap" /tmp/c2.log | sed "s/^/  gap lines: /"
done
v4l2-ctl -d /dev/video2 -c auto_exposure=3; echo "restored cam2: $(v4l2-ctl -d /dev/video2 -C auto_exposure)"
