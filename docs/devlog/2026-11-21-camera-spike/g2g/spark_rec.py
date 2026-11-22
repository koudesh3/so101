# Runs g2g_cap on both cameras, decodes each JPEG to a small grayscale image, and logs an 8x6 grid
# of mean brightness per frame, with the frame's timestamps converted to Spark wall-clock time.
# usage: python spark_rec.py OUT.jsonl [W H]   (stops on SIGTERM / Ctrl-C)
import json, os, signal, struct, subprocess, sys, threading
import cv2, numpy as np

OUT = sys.argv[1]
W, H = (sys.argv[2], sys.argv[3]) if len(sys.argv) > 3 else ('640', '480')
HERE = os.path.dirname(os.path.abspath(__file__))
CAMS = {'cam1': '/dev/video0', 'cam2': '/dev/video2'}
GX, GY = 8, 6
lock = threading.Lock()
out = open(OUT, 'w')
procs = {}

def reader(name, p):
    hdr = struct.Struct('<dddII')
    n = 0
    while True:
        h = p.stdout.read(hdr.size)
        if len(h) < hdr.size: break
        buf_mono, deq_mono, deq_real, seq, ln = hdr.unpack(h)
        jpg = p.stdout.read(ln)
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_4)
        if img is None: continue
        hh, ww = img.shape[0] // GY * GY, img.shape[1] // GX * GX
        grid = img[:hh, :ww].reshape(GY, hh // GY, GX, ww // GX).mean(axis=(1, 3))
        rec = {'cam': name, 'seq': seq, 'buf_real': buf_mono + (deq_real - deq_mono),
               'deq_real': deq_real, 'grid': [round(float(v), 1) for v in grid.ravel()]}
        with lock:
            out.write(json.dumps(rec) + '\n')
        n += 1
        if n % 300 == 0: print(f'{name}: {n} frames', flush=True)

def shutdown(*_):
    for p in procs.values(): p.terminate()
signal.signal(signal.SIGTERM, shutdown); signal.signal(signal.SIGINT, shutdown)

threads = []
for name, dev in CAMS.items():
    procs[name] = subprocess.Popen([os.path.join(HERE, 'g2g_cap'), dev, W, H, '30'], stdout=subprocess.PIPE)
    th = threading.Thread(target=reader, args=(name, procs[name])); th.start(); threads.append(th)
print('recording', flush=True)
for th in threads: th.join()
out.close()
print('stopped', flush=True)
