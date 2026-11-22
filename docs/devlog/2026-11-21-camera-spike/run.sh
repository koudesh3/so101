#!/bin/bash
# Innomaker U20CAM spike on Linux. usage: ./run.sh [long_secs=600]
set -u
cd "$(dirname "$0")"
LONG=${1:-600}
DEV=$(for d in /sys/class/video4linux/video*; do
  grep -q Innomaker $d/name && [ "$(cat $d/index)" = 0 ] && echo /dev/$(basename $d); done | head -1)
[ -z "$DEV" ] && { echo "camera not found"; exit 1; }

echo "== host"; uname -a; date -Is
echo "== device $DEV"
SYS=$(readlink -f /sys/class/video4linux/$(basename $DEV)/device/..)
echo "usb sysfs path: $SYS  (bus-port chain: $(basename $SYS))  speed: $(cat $SYS/speed) Mb/s"
echo "== lsusb -t"; lsusb -t
echo "== lsusb -v (camera)"; lsusb -v -d 0c45:6366 2>/dev/null | head -400
echo "== v4l2-ctl --list-devices"; v4l2-ctl --list-devices
echo "== v4l2-ctl --list-formats-ext"; v4l2-ctl -d $DEV --list-formats-ext
echo "== v4l2-ctl --list-ctrls-menus"; v4l2-ctl -d $DEV --list-ctrls-menus

gcc -O2 -o v4l2probe v4l2probe.c -lm || exit 1
echo "== per-mode sweep (8s each)"
for m in "640 360 30 MJPG" "640 480 30 MJPG" "800 600 30 MJPG" "1024 576 30 MJPG" "1280 720 30 MJPG" "1280 960 30 MJPG" "1920 1080 30 MJPG" \
         "640 360 30 YUYV" "640 480 30 YUYV" "1280 720 10 YUYV" "1280 720 5 YUYV" "1920 1080 5 YUYV"; do
  echo "-- $m"; ./v4l2probe $DEV $m 8 2>&1 | grep -v "^  gap"
done
for m in "1920 1080 30 MJPG" "640 480 30 MJPG"; do
  echo "== long run ${LONG}s: $m"; ./v4l2probe $DEV ${m% *} $LONG ${m##* }
done
