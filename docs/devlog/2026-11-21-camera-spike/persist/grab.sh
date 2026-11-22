#!/bin/bash
# grab.sh LABEL [DEV]: capture 20 MJPEG frames, print current controls and the last frame's mean B/G/R in the center
cd "$(dirname "$0")"; D=${2:-/dev/video0}
ctl=$(v4l2-ctl -d $D -C auto_exposure,exposure_time_absolute,white_balance_automatic,white_balance_temperature,gain | sed "s/ (.*)//" | tr "\n" " ")
v4l2-ctl -d $D --set-fmt-video=width=640,height=480,pixelformat=MJPG --stream-mmap --stream-count=20 --stream-to=/tmp/g.mjpg 2>/dev/null
/projects/lerobot/.venv/bin/python - "$1" "$ctl" <<"PY"
import sys, cv2, numpy as np
d = open("/tmp/g.mjpg", "rb").read(); s = d.rfind(b"\xff\xd8")
img = cv2.imdecode(np.frombuffer(d[s:], np.uint8), cv2.IMREAD_COLOR)
c = img[120:360, 160:480].reshape(-1, 3).mean(0)
print(f"{sys.argv[1]:<30} B {c[0]:6.1f} G {c[1]:6.1f} R {c[2]:6.1f}  R/B {c[2]/max(c[0],1):5.2f} | {sys.argv[2]}")
PY
