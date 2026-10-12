"""SVG: how much of a full turn each SO-101 joint can actually use.

Each servo could turn all the way round; the arm's own links stop it first. The limits are
the follower URDF's (as packed into src/report/assets/model.json, which the 3D views draw), and
the colours are the dashboard's joint colours, computed the same way (dashboard.html: LCH,
darkened until they clear 3:1 against the paper).

    python3 figures/joint_ranges.py > joint-ranges.svg
"""
import json
import math
import sys
from pathlib import Path

MODEL = Path(__file__).resolve().parent.parent / "src/report/assets/model.json"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
LCH_LIGHT = [[48, 60, 262], [68, 62, 60], [46, 62, 320], [66, 45, 190], [52, 65, 28], [70, 60, 110]]
INK, MID, FAINT, TRACK = "#121212", "#555555", "#8c8c8c", "#e3e3e3"


# ---------- colour, as dashboard.html ----------

def lch(L, C, H):
    h = math.radians(H)
    a, b = C * math.cos(h), C * math.sin(h)
    fy = (L + 16) / 116
    fx, fz = fy + a / 500, fy - b / 200
    e, k = 216 / 24389, 24389 / 27
    f = lambda t: t ** 3 if t ** 3 > e else (116 * t - 16) / k
    X, Y, Z = 0.95047 * f(fx), (fy ** 3 if L > k * e else L / k), 1.08883 * f(fz)
    lin = [3.2404542 * X - 1.5371385 * Y - 0.4985314 * Z,
           -0.969266 * X + 1.8760108 * Y + 0.041556 * Z,
           0.0556434 * X - 0.2040259 * Y + 1.0572252 * Z]
    out = []
    for u in lin:
        u = min(1.0, max(0.0, u))
        out.append(12.92 * u if u <= 0.0031308 else 1.055 * u ** (1 / 2.4) - 0.055)
    return out


def rel_lum(c):
    lin = [v / 12.92 if v <= 0.03928 else ((v + 0.055) / 1.055) ** 2.4 for v in c]
    return 0.2126 * lin[0] + 0.7152 * lin[1] + 0.0722 * lin[2]


def legible(l, c, h, minimum=3.05):
    col = lch(l, c, h)
    for _ in range(60):
        if (1.05) / (rel_lum(col) + 0.05) >= minimum:   # against white paper
            break
        l -= 1
        col = lch(l, c, h)
    return "#" + "".join(f"{round(v * 255):02x}" for v in col)


COLORS = [legible(*x) for x in LCH_LIGHT]


# ---------- limits, from the model the 3D views draw ----------

joints = {j["name"]: j for j in json.loads(MODEL.read_text())["arms"]["follower"]["joints"]}
limits = [(math.degrees(joints[n]["lower"]), math.degrees(joints[n]["upper"])) for n in JOINTS]


def f(v):
    return f"{v:.1f}".rstrip("0").rstrip(".")


def deg(v):
    s = f"{v:.1f}".rstrip("0").rstrip(".")
    return s.replace("-", "−") + "°"


# Zero at the top, positive counterclockwise, as in the encoder figure: the part the arm blocks
# opens at the bottom.
def at(cx, cy, r, a):
    t = math.radians(a)
    return cx - r * math.sin(t), cy - r * math.cos(t)


def arc(cx, cy, r, lo, hi):
    (x0, y0), (x1, y1) = at(cx, cy, r, lo), at(cx, cy, r, hi)
    large = 1 if hi - lo > 180 else 0
    return f"M{f(x0)} {f(y0)} A{f(r)} {f(r)} 0 {large} 0 {f(x1)} {f(y1)}"


W, H = 720, 244
R, PITCH, CY = 40.0, 120.0, 86.0
parts = []
for i, (name, (lo, hi), col) in enumerate(zip(JOINTS, limits, COLORS)):
    cx = PITCH * (i + 0.5)
    span = hi - lo
    (ix, iy), (zx, zy) = at(cx, CY, R - 5, 0), at(cx, CY, R + 7, 0)
    parts += [
        f'<circle cx="{f(cx)}" cy="{f(CY)}" r="{f(R)}" fill="none" stroke="{TRACK}" stroke-width="6"/>',
        f'<path d="{arc(cx, CY, R, lo, hi)}" fill="none" stroke="{col}" stroke-width="6"/>',
        # the zero pose
        f'<path d="M{f(ix)} {f(iy)} L{f(zx)} {f(zy)}" stroke="{INK}" stroke-width="1"/>',
        f'<text class="m" x="{f(cx)}" y="{f(CY + 1)}" font-size="15" text-anchor="middle" '
        f'dominant-baseline="central" fill="{INK}">{round(span / 360 * 100)}%</text>',
        # bus ID chip and name, as the 3D views tag the servos
        f'<rect x="{f(cx - 52)}" y="{f(CY + R + 24)}" width="13" height="13" fill="{col}"/>',
        f'<text class="m" x="{f(cx - 45.5)}" y="{f(CY + R + 31)}" font-size="9" font-weight="500" '
        f'text-anchor="middle" dominant-baseline="central" fill="#ffffff">{i + 1}</text>',
        f'<text x="{f(cx - 34)}" y="{f(CY + R + 31)}" font-size="10" letter-spacing="0.8" '
        f'dominant-baseline="central" fill="{INK}">{name.replace("_", " ").upper()}</text>',
        f'<text class="m" x="{f(cx)}" y="{f(CY + R + 52)}" font-size="10" text-anchor="middle" '
        f'fill="{MID}">{deg(lo)} to {deg(hi)}</text>',
        f'<text class="m" x="{f(cx)}" y="{f(CY + R + 67)}" font-size="10" text-anchor="middle" '
        f'fill="{FAINT}">{deg(span)} of 360°</text>',
    ]

label = "; ".join(f"{n.replace('_', ' ')} {round((hi - lo) / 360 * 100)}% ({deg(hi - lo)})"
                  for n, (lo, hi) in zip(JOINTS, limits))
sys.stdout.write(f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-size="12" role="img" aria-label="How much of a full turn each SO-101 joint can use before the arm stops it: {label}."><style>text{{font-family:-apple-system, BlinkMacSystemFont, 'SF Pro Text', 'Inter', 'Helvetica Neue', Arial, sans-serif}}.m{{font-family:'SF Mono', ui-monospace, 'JetBrains Mono', Menlo, Consolas, monospace}}</style>
<rect width="{W}" height="{H}" fill="#ffffff"/>
{chr(10).join(parts)}
</svg>
''')
