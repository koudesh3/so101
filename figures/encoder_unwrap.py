"""Animated SVG: the servo's turn (0..2pi rad) opening into its encoder's number line (0..4096).

The curve keeps its length (2*pi*R) while its curvature eases from 1/R to 0, pinned at its
midpoint, so the circle opens at the top and flattens into a line. SMIL interpolates between
precomputed keyframes; every animated path keeps the same command structure.
"""
import math
import sys

W, H = 720, 410
R = 96.0
L = 2 * math.pi * R
CX, YB = W / 2, 330.0          # midpoint of the curve (2048) sits here, tangent horizontal
PERIOD = 9.0                   # seconds per loop
N = 96                        # polyline points on the curve
FRAMES = 22                    # keyframes for the opening
INK, MID, FAINT, RED = "#121212", "#555555", "#8c8c8c", "#db484b"


def ease(u):
    return u * u * (3 - 2 * u)


def point(s, k):
    """Position and outward normal at arc length s, curvature k (screen coords, y down)."""
    t = s - L / 2
    if k < 1e-9:
        return (CX + t, YB), (0.0, 1.0)
    phi = k * t
    x = CX + math.sin(phi) / k
    y = YB - (1 - math.cos(phi)) / k
    # tangent (cos phi, -sin phi); outward normal points away from the centre of curvature
    return (x, y), (math.sin(phi), math.cos(phi))


def f(v):
    return f"{v:.1f}".rstrip("0").rstrip(".")


def curve_d(k):
    pts = [point(L * i / N * (4095 / 4096), k)[0] for i in range(N + 1)]
    return "M" + " L".join(f"{f(x)} {f(y)}" for x, y in pts)


def ticks_d(k, step, length, stop=4096):
    out = []
    for v in range(0, stop, step):
        (x, y), (nx, ny) = point(L * v / 4096, k)
        out.append(f"M{f(x)} {f(y)} L{f(x + nx * length)} {f(y + ny * length)}")
    return " ".join(out)


def label_pos(v, k, off):
    (x, y), (nx, ny) = point(L * v / 4096, k)
    return x + nx * off, y + ny * off


# Timeline: hold the circle, open it, hold the line, close it.
HOLD_A, OPEN, HOLD_B, CLOSE = 4.0, 1.2, 2.75, 1.05   # 75% of the loop holding, the circle longest
assert abs(HOLD_A + OPEN + HOLD_B + CLOSE - PERIOD) < 1e-9
curv = [ease(1 - i / (FRAMES - 1)) / R for i in range(FRAMES)]   # 1/R -> 0
times, ks = [0.0], [curv[0]]
clock = HOLD_A


def phase(duration, states):
    global clock
    for i, st in enumerate(states):
        times.append(clock + duration * i / (len(states) - 1)); ks.append(st)
    clock += duration


phase(OPEN, curv)
clock += HOLD_B
phase(CLOSE, list(reversed(curv)))
times.append(PERIOD); ks.append(curv[0])
key_times = ";".join(f"{t / PERIOD:.4f}" for t in times)


def anim(attr, values):
    return (f'<animate attributeName="{attr}" dur="{PERIOD}s" repeatCount="indefinite" '
            f'calcMode="linear" keyTimes="{key_times}" values="{";".join(values)}"/>')


def moving_text(v, off, text, cls, fill, anchor="middle", size=11, fade=None):
    xs = [label_pos(v, k, off) for k in ks]
    x0, y0 = xs[0]
    extra = ""
    if fade:
        # opacity follows how open the curve is
        ops = [f(1 - k * R) if fade == "in" else f(k * R) for k in ks]
        extra = anim("opacity", ops)
        op0 = ops[0]
    else:
        op0 = "1"
    return (f'<text class="{cls}" x="0" y="0" font-size="{size}" text-anchor="{anchor}" '
            f'dominant-baseline="central" fill="{fill}" opacity="{op0}" '
            f'transform="translate({f(x0)} {f(y0)})">{text}'
            f'<animateTransform attributeName="transform" type="translate" dur="{PERIOD}s" '
            f'repeatCount="indefinite" calcMode="linear" keyTimes="{key_times}" '
            f'values="{";".join(f"{f(x)} {f(y)}" for x, y in xs)}"/>{extra}</text>')


def static_text(v, off, text, cls, fill, k, size=11):
    x, y = label_pos(v, k, off)
    return (f'<text class="{cls}" x="{f(x)}" y="{f(y)}" font-size="{size}" text-anchor="middle" '
            f'dominant-baseline="central" fill="{fill}">{text}</text>')


def layer(animated):
    k0 = 0.0  # static fallback shows the open line
    parts = []
    def path(dfn, stroke, width):
        if animated:
            return (f'<path d="{dfn(ks[0])}" fill="none" stroke="{stroke}" stroke-width="{width}" '
                    f'stroke-linecap="round">{anim("d", [dfn(k) for k in ks])}</path>')
        return f'<path d="{dfn(k0)}" fill="none" stroke="{stroke}" stroke-width="{width}" stroke-linecap="round"/>'
    parts.append(path(lambda k: ticks_d(k, 64, 4), FAINT, 1))
    parts.append(path(lambda k: ticks_d(k, 512, 7), MID, 1))
    parts.append(path(lambda k: ticks_d(k, 1024, 11), INK, 1.25))
    parts.append(path(curve_d, INK, 1.5))
    # The far end's tick, which sits on top of 0 while the curve is closed.
    end_tick = lambda k: ticks_d(k, 4096, 11, stop=4097).split(" M")[-1].lstrip("M")
    if animated:
        ops = [f(1 - k * R) for k in ks]
        parts.append(f'<path d="M{end_tick(ks[0])}" fill="none" stroke="{INK}" stroke-width="1.25" opacity="{ops[0]}">'
                     f'{anim("d", ["M" + end_tick(k) for k in ks])}{anim("opacity", ops)}</path>')
    else:
        parts.append(f'<path d="M{end_tick(k0)}" fill="none" stroke="{INK}" stroke-width="1.25"/>')
    rad = {0: "0", 1024: "π/2", 2048: "π", 3072: "3π/2"}
    for v, txt in rad.items():
        parts.append(moving_text(v, -16, txt, "m", RED, size=11) if animated
                     else static_text(v, -16, txt, "m", RED, k0))
    for v in (0, 1024, 2048, 3072):
        parts.append(moving_text(v, 25, str(v), "m", INK) if animated
                     else static_text(v, 25, str(v), "m", INK, k0))
    # The far end only gets its own labels once the ends have come apart.
    end = 4096
    if animated:
        parts.append(moving_text(end, -16, "2π", "m", RED, fade="in"))
        parts.append(moving_text(end, 25, "4096", "m", INK, fade="in"))
    else:
        parts.append(static_text(end, -16, "2π", "m", RED, k0))
        parts.append(static_text(end, 25, "4096", "m", INK, k0))
    return "\n".join(parts)


# ---------- A position dot on its own, slower loop, and the register value it reads ----------

BLUE = "#007cd7"
TURN = 13.5     # seconds per turn of the dot
TOTAL = 27.0    # both loops line up again: 3 curve loops, 2 dot turns
FPS = 20        # readout refresh rate
M = int(TOTAL * FPS)


def curve_at(t):
    """The curve's state at time t, as the two keyframes SMIL is blending and how far between."""
    t %= PERIOD
    for i in range(len(times) - 1):
        if times[i] <= t <= times[i + 1]:
            span = times[i + 1] - times[i]
            return ks[i], ks[i + 1], (t - times[i]) / span if span else 0.0
    return ks[-1], ks[-1], 0.0


def dot_xy(t, v):
    ka, kb, a = curve_at(t)
    (xa, ya), _ = point(L * v / 4096, ka)
    (xb, yb), _ = point(L * v / 4096, kb)
    return xa + (xb - xa) * a, ya + (yb - ya) * a


def value_at(t):
    return t / TURN % 1 * 4096


# Dot keyframes. At each wrap it is at 4096 and 0 in the same instant: one point on the
# wheel, the two ends of the line.
wrap_every = int(TURN * FPS)
dot_keys = []
for j in range(M + 1):
    t = j / FPS
    if j == M:
        dot_keys.append((t, dot_xy(t, 4096)))
        break
    if j and j % wrap_every == 0:
        dot_keys.append((t, dot_xy(t, 4096)))
    dot_keys.append((t, dot_xy(t, value_at(t))))

readings = [int(value_at(j / FPS)) for j in range(M)]
readout_kt = ";".join(f"{j / M:.4f}" for j in range(M))


def dot(animated):
    if not animated:
        x, y = point(L * 1024 / 4096, 0.0)[0]
        return f'<circle cx="{f(x)}" cy="{f(y)}" r="4.5" fill="{BLUE}" stroke="#ffffff" stroke-width="2"/>'
    return (f'<circle r="4.5" fill="{BLUE}" stroke="#ffffff" stroke-width="2">'
            f'<animateTransform attributeName="transform" type="translate" dur="{TOTAL}s" '
            f'repeatCount="indefinite" calcMode="linear" '
            f'keyTimes="{";".join(f"{t / TOTAL:.4f}" for t, _ in dot_keys)}" '
            f'values="{";".join(f"{f(x)} {f(y)}" for _, (x, y) in dot_keys)}"/></circle>')


# The angle, drawn only while the curve is (nearly) a circle: centre, radius to the dot, theta.
GEO_FPS = 10
GEO_M = int(TOTAL * GEO_FPS)
ARC_R, ARC_PTS = 22.0, 20


def geometry(t, v):
    ka, kb, a = curve_at(t)
    k = max(ka + (kb - ka) * a, 0.5 / R)   # invisible below this; keeps the centre on the page
    cx, cy = CX, YB - 1 / k
    phi0, phi = -k * L / 2, k * (L * v / 4096 - L / 2)

    def around(p, r):
        return cx + r * math.sin(p), cy + r * math.cos(p)

    arc = [around(phi0 + (phi - phi0) * i / (ARC_PTS - 1), ARC_R) for i in range(ARC_PTS)]
    label = around((phi0 + phi) / 2, 34)
    opacity = min(max((k * R - 0.7) / 0.3, 0.0), 1.0)
    return (cx, cy), dot_xy(t, v), arc, label, opacity


geo_keys = []
for j in range(GEO_M + 1):
    t = j / GEO_FPS
    if j == GEO_M:
        geo_keys.append((t, geometry(t, 4096)))
        break
    if j and j % int(TURN * GEO_FPS) == 0:
        geo_keys.append((t, geometry(t, 4096)))
    geo_keys.append((t, geometry(t, value_at(t))))


def angle(animated):
    if not animated:
        return ""  # the still frame is the open line, where there is no centre
    kt = ";".join(f"{t / TOTAL:.4f}" for t, _ in geo_keys)

    def a(attr, vals):
        return (f'<animate attributeName="{attr}" dur="{TOTAL}s" repeatCount="indefinite" '
                f'calcMode="linear" keyTimes="{kt}" values="{";".join(vals)}"/>')

    def tr(vals):
        return (f'<animateTransform attributeName="transform" type="translate" dur="{TOTAL}s" '
                f'repeatCount="indefinite" calcMode="linear" keyTimes="{kt}" values="{";".join(vals)}"/>')

    g = [k for _, k in geo_keys]
    (c0, d0, arc0, lab0, op0) = g[0]
    radius = [f"M{f(c[0])} {f(c[1])} L{f(d[0])} {f(d[1])}" for c, d, *_ in g]
    arcs = ["M" + " L".join(f"{f(x)} {f(y)}" for x, y in arc) for _, _, arc, *_ in g]
    return (f'<g opacity="{f(op0)}">{a("opacity", [f(k[4]) for k in g])}'
            f'<path d="{radius[0]}" stroke="{FAINT}" stroke-width="1" fill="none">{a("d", radius)}</path>'
            f'<path d="{arcs[0]}" stroke="{RED}" stroke-width="1.25" fill="none">{a("d", arcs)}</path>'
            f'<circle r="2.5" fill="{FAINT}">{tr([f"{f(c[0])} {f(c[1])}" for c, *_ in g])}</circle>'
            f'<text x="0" y="0" font-size="13" font-style="italic" text-anchor="middle" '
            f'dominant-baseline="central" fill="{RED}">θ{tr([f"{f(k[3][0])} {f(k[3][1])}" for k in g])}</text>'
            f'</g>')


def cell(x, y, glyph, size, fill):
    return f'<text class="m" x="{f(x)}" y="{f(y)}" font-size="{size}" text-anchor="middle" fill="{fill}">{glyph}</text>'


clip_ids = iter(range(1000))


def strip(x, y, glyphs, fills, idxs, size, animated):
    """One character cell that shows glyphs[idx], stepping through idxs over the dot's loop."""
    if not animated:
        return cell(x, y, glyphs[idxs[0]], size, fills[idxs[0]])
    lh = size * 1.25
    cid = f"c{next(clip_ids)}"
    texts = "".join(cell(x, y + i * lh, g, size, fills[i]) for i, g in enumerate(glyphs))
    return (f'<clipPath id="{cid}"><rect x="{f(x - size)}" y="{f(y - size * 0.85)}" '
            f'width="{f(size * 2)}" height="{f(size * 1.15)}"/></clipPath>'
            f'<g clip-path="url(#{cid})"><g>{texts}'
            f'<animateTransform attributeName="transform" type="translate" dur="{TOTAL}s" '
            f'repeatCount="indefinite" calcMode="discrete" keyTimes="{readout_kt}" '
            f'values="{";".join(f"0 {f(-i * lh)}" for i in idxs)}"/></g></g>')


def readout(animated):
    """A four-row table in the top left: the dot's position as the register holds it."""
    vals = readings if animated else [1024]
    x0, y0, row, size, pitch = 40.0, 40.0, 20.0, 12, 7.6
    xv = x0 + 34                     # value column
    width = 34 + 16 * pitch + 6
    out = [f'<path d="M{f(x0)} {f(y0 - 14)} H{f(x0 + width)}" stroke="{INK}" stroke-width="1"/>']
    for r in (1, 2, 3, 4):
        y = y0 - 14 + row * r
        out.append(f'<path d="M{f(x0)} {f(y)} H{f(x0 + width)}" stroke="#e3e3e3" stroke-width="1"/>')
    for r, name in enumerate(("θ", "dec", "bin", "hex")):
        out.append(f'<text x="{f(x0)}" y="{f(y0 + row * r)}" font-size="11" fill="{MID}">{name}</text>')

    def at(col):
        return xv + pitch * (col + .5)

    # θ: the domain, 0 to 2π, in radians to two places
    y = y0
    hund = [round(v / 4096 * 2 * math.pi * 100) % 629 for v in vals]
    for col, place in ((0, 100), (2, 10), (3, 1)):
        out.append(strip(at(col), y, "0123456789", [RED] * 10, [h // place % 10 for h in hund], size, animated))
    out.append(cell(at(1), y, ".", size, RED))
    out.append(f'<text class="m" x="{f(at(4) - pitch / 2 + 4)}" y="{f(y)}" font-size="11" fill="{MID}">rad</text>')
    # dec: right-aligned in four cells
    y = y0 + row
    for d in range(4):
        p = 10 ** (3 - d)
        idxs = [0 if (v < p and d < 3) else v // p % 10 + 1 for v in vals]
        out.append(strip(at(d), y, " 0123456789", [INK] * 11, idxs, size, animated))
    # bin: twelve bits in nibbles, Rust literal style; zeros recede
    y = y0 + row * 2
    col = 0
    for ch in "0b":
        out.append(cell(at(col), y, ch, size, MID)); col += 1
    for b in range(12):
        if b and b % 4 == 0:
            out.append(cell(at(col), y, "_", size, FAINT)); col += 1
        idxs = [(v >> (11 - b)) & 1 for v in vals]
        out.append(strip(at(col), y, "01", [FAINT, INK], idxs, size, animated)); col += 1
    # hex: three nibbles, one per group of four bits above
    y = y0 + row * 3
    for i, ch in enumerate("0x"):
        out.append(cell(at(i), y, ch, size, MID))
    for n in range(3):
        idxs = [(v >> (8 - 4 * n)) & 0xF for v in vals]
        out.append(strip(at(n + 2), y, "0123456789ABCDEF", [INK] * 16, idxs, size, animated))
    return "\n".join(out)


lx, rx = CX - L / 2, CX + L / 2
svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-size="12" role="img" aria-label="One full turn of the servo, 0 to 2π radians, unrolls into the encoder's number line of 4096 positions: π/2 is 1024, π is 2048, 3π/2 is 3072. Zero is at the top and angles run counterclockwise. A dot circles the turn while a small table shows its position as an angle in radians and as the register value in decimal, binary and hex."><style>text{{font-family:-apple-system, BlinkMacSystemFont, 'SF Pro Text', 'Inter', 'Helvetica Neue', Arial, sans-serif}}.m{{font-family:'SF Mono', ui-monospace, 'JetBrains Mono', Menlo, Consolas, monospace}}.still{{display:none}}@media (prefers-reduced-motion: reduce){{.moving{{display:none}}.still{{display:inline}}}}</style>
<rect width="{W}" height="{H}" fill="#ffffff"/>
<g class="moving">
{layer(True)}
{angle(True)}
{dot(True)}
{readout(True)}
</g>
<g class="still">
{layer(False)}
{dot(False)}
{readout(False)}
</g>
<text x="{f(lx)}" y="{H - 14}" font-size="11" fill="{MID}">radians, one turn</text>
<text x="{f(rx)}" y="{H - 14}" font-size="11" text-anchor="end" fill="{MID}">encoder ticks, 12 bits</text>
<rect x="{f(lx - 22)}" y="{H - 18}" width="12" height="2" fill="{RED}"/>
<rect x="{f(rx + 10)}" y="{H - 18}" width="12" height="2" fill="{INK}"/>
</svg>
'''
sys.stdout.write(svg)
