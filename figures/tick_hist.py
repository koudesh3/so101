"""How long each tick spends on the buses, as two histograms, from a recorded byte log.

    python3 figures/tick_hist.py episodes/grab1.jsonl.gz tick-time.svg

A tick's bus time runs from the follower's sync read going out, through the leader's read, to:
  with telemetry      the reply to the dashboard's telemetry read, the tick's last packet;
  without telemetry   the end of the follower's sync write, which gets no reply, so its send time
                      plus its 26 bytes on the wire (260 us). The telemetry read comes after it,
                      so leaving it out changes nothing before.
Each is drawn against its bytes alone at 1 Mbaud (both buses, one after the other: 150 and 179
bytes) and the 10 ms period at 100 Hz. Standard library only; drawn like the post's other diagrams.
"""
import gzip
import json
import statistics
import sys
from pathlib import Path

src, dst = sys.argv[1], sys.argv[2]
rows = [json.loads(line) for line in (gzip.open if src.endswith(".gz") else open)(src, "rt")]
BYTE_US = 10  # 1 Mbaud, 10 bits a byte


def instr(r):
    return r["hex"][8:10]


def is_tx(r, port, code):
    return r["port"] == port and r["dir"] == "tx" and instr(r) == code


starts = [i for i, r in enumerate(rows) if is_tx(r, "follower", "82")]
with_t, without_t = [], []
for a, b in zip(starts, starts[1:]):
    seg = rows[a:b]
    write = next((r for r in seg if is_tx(r, "follower", "83")), None)
    if write is None:
        continue
    t0 = seg[0]["t_us"]
    with_t.append((seg[-1]["t_us"] - t0) / 1000)
    without_t.append((write["t_us"] + 26 * BYTE_US - t0) / 1000)

INK, INK2, INK3, RULE = "#121212", "#555555", "#737373", "#d9d9d9"
SANS = "-apple-system, BlinkMacSystemFont, 'SF Pro Text', 'Inter', 'Helvetica Neue', Arial, sans-serif"
SERIES = [  # label, spans, bytes per tick, colour
    ("without telemetry", without_t, 14 + 48 + 26 + 14 + 48, "#007cd7"),
    ("with the dashboard's telemetry", with_t, 14 + 48 + 26 + 8 + 21 + 14 + 48, "#994db0"),
]

W, H = 720, 330
L, R, T, B = 66, 690, 82, 280   # plot box
XMAX, BIN = 10.0, 0.25          # ms; ticks past XMAX are counted, not drawn
nb = int(XMAX / BIN)


def hist(spans):
    counts = [0] * (nb + 1)
    for s in spans:
        counts[min(int(s / BIN), nb)] += 1
    return counts


hists = [hist(s) for _, s, _, _ in SERIES]
peak = max(max(h[:nb]) for h in hists)
ymax = (peak // 200 + 1) * 200


def X(ms):
    return L + (R - L) * ms / XMAX


def Y(n):
    return B - (B - T) * n / ymax


out = []


def text(x, y, s, size=11, anchor="middle", fill=INK2, weight=400, spacing=None):
    sp = f' letter-spacing="{spacing}"' if spacing is not None else ""
    out.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" text-anchor="{anchor}" '
               f'font-weight="{weight}" fill="{fill}"{sp}>{s}</text>')


def vline(ms, colour, dash=None, top=T):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<path d="M{X(ms):.1f} {top}V{B}" stroke="{colour}" stroke-width="1.5"{d}/>')


out.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
for n in range(0, ymax + 1, 200):
    out.append(f'<path d="M{L} {Y(n):.1f}H{R}" stroke="{RULE}" stroke-width="1"/>')
    text(L - 8, Y(n) + 4, f"{n:,}", 10, "end", INK3)

# Each distribution as a filled step outline, so the two read through each other.
for (_, _, _, colour), counts in zip(SERIES, hists):
    d = f"M{X(0):.1f} {B}"
    for i, n in enumerate(counts[:nb]):
        d += f"V{Y(n):.1f}H{X((i + 1) * BIN):.1f}"
    d += f"V{B}Z"
    out.append(f'<path d="{d}" fill="{colour}" fill-opacity="0.18" stroke="{colour}" stroke-width="1.5" stroke-linejoin="round"/>')

out.append(f'<path d="M{L} {B}H{R}" stroke="{INK}" stroke-width="1"/>')
out.append(f'<path d="M{L} {T}V{B}" stroke="{INK}" stroke-width="1"/>')
for n in range(0, ymax + 1, 200):
    out.append(f'<path d="M{L - 4} {Y(n):.1f}H{L}" stroke="{INK}" stroke-width="1"/>')
out.append(f'<text x="16" y="{(T + B) / 2:.1f}" font-size="10" text-anchor="middle" fill="{INK3}" '
           f'letter-spacing="0.8" transform="rotate(-90 16 {(T + B) / 2:.1f})">TICKS</text>')
for ms in range(0, int(XMAX) + 1):
    out.append(f'<path d="M{X(ms):.1f} {B}V{B + 4}" stroke="{INK}" stroke-width="1"/>')
    text(X(ms), B + 17, f"{ms}", 10, fill=INK3)
text((L + R) / 2, B + 38, "BUS TIME PER TICK, MS", 10, fill=INK3, spacing=0.8)

# Markers: dashed at the bytes alone, solid at the median; the legend gives the numbers.
for row, (name, spans, nbytes, colour) in enumerate(SERIES):
    wire = nbytes * BYTE_US / 1000
    med = statistics.median(spans)
    p90 = sorted(spans)[9 * len(spans) // 10]
    vline(wire, colour, "4 3")
    vline(med, colour)
    y = 22 + row * 20
    out.append(f'<rect x="{L}" y="{y - 9}" width="12" height="10" fill="{colour}" fill-opacity="0.18" stroke="{colour}" stroke-width="1.5"/>')
    text(L + 20, y, name, 11, "start", INK, 500)
    text(L + 210, y, f"median {med:.1f} ms · p90 {p90:.1f} ms · bytes alone {wire:.2f} ms", 11, "start", INK2)
y = 22 + len(SERIES) * 20
out.append(f'<path d="M{L} {y - 4}h12" stroke="{INK2}" stroke-width="1.5" stroke-dasharray="4 3"/>')
text(L + 20, y, "bytes alone", 11, "start", INK2)
out.append(f'<path d="M{L + 110} {y - 4}h12" stroke="{INK2}" stroke-width="1.5"/>')
text(L + 130, y, "median", 11, "start", INK2)

vline(XMAX, INK3, "2 3")
over = hists[1][nb]
text(X(XMAX) - 6, T + 14, "one tick at 100 Hz, 10 ms", 11, "end", INK2)
text(X(XMAX) - 6, T + 28, f"{over} of {len(with_t):,} ticks ran past it", 11, "end", INK3)

meds = [statistics.median(s) for _, s, _, _ in SERIES]
label = (f"Two histograms of the time each of {len(with_t):,} ticks spent on the servo buses: "
         f"without the dashboard's telemetry read, median {meds[0]:.1f} ms against 1.5 ms of bytes; "
         f"with it, median {meds[1]:.1f} ms against 1.79 ms of bytes. A tick at 100 Hz is 10 ms.")
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" '
       f'aria-label="{label}"><style>text{{font-family:{SANS}}}</style>\n' + "\n".join(out) + "\n</svg>\n")
Path(dst).write_text(svg)
for (name, spans, nbytes, _), m in zip(SERIES, meds):
    print(f"{name}: median {m:.2f} ms, bytes {nbytes * BYTE_US / 1000:.2f} ms")
print("wrote", dst)
