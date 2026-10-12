"""The wiring figure: build_scene.py's render with labels laid over it in the 3D arm's style.

    python3 compose.py render.json labels.svg [follower|both] [text scale]

A text scale above 1 sets the labels larger at the same anchor points (the post's phone version uses
1.7, as follower-labels-narrow.svg, and leaves the chip inset out).

Labels are the viewer's tags (small uppercase, letter-spaced, a coloured number chip for each servo,
grey leader lines), the CH343P inset is boxed in ink on paper like the post's other
diagrams. Positions come from gen_wiring.py's projection P(), which the render's camera matches.
Standard library only. The SVG is the labels alone, on a transparent ground, the same size as the render:
wiring.html lays it over the dithered render.
"""
import json
import math
import sys
from pathlib import Path

render, dst = sys.argv[1], sys.argv[2]
variant = sys.argv[3] if len(sys.argv) > 3 else "follower"
TS = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
colours = json.loads(Path(render).read_text())

sys.path.insert(0, str(Path(__file__).resolve().parent))
from layout import S, VARIANTS  # noqa: E402

LAYOUT = VARIANTS[variant]
OX, OY, W, H = LAYOUT["OX"], LAYOUT["OY"], LAYOUT["W"], LAYOUT["H"]
C = math.cos(math.radians(30))
INK, INK2, INK3, RULE, PAPER = "#121212", "#555555", "#737373", "#d9d9d9", "#ffffff"
SANS = "-apple-system, BlinkMacSystemFont, 'SF Pro Text', 'Inter', 'Helvetica Neue', Arial, sans-serif"
MONO = "'SF Mono', ui-monospace, 'JetBrains Mono', Menlo, Consolas, monospace"
# The figure shows at roughly 0.45 of its size in the post's column, so 22 units read as the
# viewer's 10 px tags.
TAG = 22 * TS


def P(x, y, z=0):
    return (OX - (x - y) * C * S, OY + ((x + y) * 0.5 - z) * S)


out = []


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text(x, y, s, size=TAG, anchor="start", fill=INK, cls="t", weight=400, spacing=None):
    sp = f' letter-spacing="{spacing}"' if spacing is not None else ""
    out.append(f'<text class="{cls}" x="{x:.1f}" y="{y:.1f}" font-size="{size:g}" font-weight="{weight}" '
               f'text-anchor="{anchor}" fill="{fill}"{sp}>{esc(s)}</text>')


def tag(x, y, s, anchor="start", chip=None, colour=None):
    """A viewer tag: uppercase, letter-spaced, on a paper halo; with a numbered chip before it."""
    width = len(s) * (TAG * 0.66 + 1.8 * TS)
    # Larger type (TS > 1) is kept inside the canvas on the left too: a tag ending at x (chip and all)
    # that would start past the left edge moves right until it fits.
    if anchor == "end" and TS != 1:
        reach = width + (28 * TS + 12 * TS if chip is not None else 0)
        x = max(x, 16 + reach)
    if chip is not None:
        c = 28 * TS
        cx = x if anchor == "start" else x - c - 2
        out.append(f'<rect x="{cx:.1f}" y="{y - 21 * TS:.1f}" width="{c:g}" height="{c:g}" fill="{colour}"/>')
        out.append(f'<text class="m" x="{cx + c / 2:.1f}" y="{y - TS:.1f}" font-size="{19 * TS:g}" font-weight="500" '
                   f'text-anchor="middle" fill="#fff">{chip}</text>')
        x = x + c + 10 * TS if anchor == "start" else x - c - 12 * TS
    # Kept inside the canvas: a tag that would run past the right edge ends at the edge instead.
    if anchor == "start" and x + width > W - 16:
        x, anchor = W - 16, "end"
    text(x, y, s.upper(), TAG, anchor, INK, "t halo", spacing=1.8 * TS)


def leader(a, b):
    out.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{INK3}" stroke-width="2"/>')


def sub(x, y, s, anchor="start", mono=False):
    width = len(s) * 19 * TS * 0.5
    if TS != 1:   # (the desktop labels were placed by hand to fit, and stay as they are)
        x = min(x, W - 16 - width) if anchor == "start" else max(x, 16 + width)
    text(x, y, s, 19 * TS, anchor, INK2, "m halo" if mono else "t halo")


# ---------- Tags ----------

SW, SL = 24.8, 45.4
X = [100 + 44 * i for i in range(6)]
names = ["shoulder pan", "shoulder lift", "elbow flex", "wrist flex", "wrist roll", "gripper"]
xr, yl, BH = -85.0, -60.0 + LAYOUT.get("mac_dy", 0), 11.0
two = len(LAYOUT["arms"]) > 1
for arm in LAYOUT["arms"]:
    dx, dy = arm["dx"], arm["dy"]
    for i, x in enumerate(X):
        top = P(x + SW / 2 + dx, dy, SL + 1)
        if arm["names"]:
            lx, ly = top[0] - 30, top[1] - 34
            leader((top[0] - 4, top[1] - 4), (lx + 2, ly + 8))
            tag(lx - 16, ly, names[i], "end", chip=i + 1, colour=colours["servos"][i])
        else:
            c = 28 * TS
            out.append(f'<rect x="{top[0] - 16 - c:.1f}" y="{top[1] - 22 - c:.1f}" width="{c:g}" height="{c:g}" fill="{colours["servos"][i]}"/>')
            out.append(f'<text class="m" x="{top[0] - 16 - c / 2:.1f}" y="{top[1] - 22 - c + 20 * TS:.1f}" font-size="{19 * TS:g}" font-weight="500" '
                       f'text-anchor="middle" fill="#fff">{i + 1}</text>')
            leader((top[0] - 4, top[1] - 4), (top[0] - 18, top[1] - 22))
    bp = P(4 + dx, 25 + dy, 0)
    leader((bp[0] - 4, bp[1] + 8), (bp[0] + 46, bp[1] + 46))
    board = f"{arm['name']}'s driver board" if two else "Servo driver board"
    tag(bp[0] + 30, bp[1] + 72, board)
    if not two:
        sub(bp[0] + 30, bp[1] + 72 + 26 * TS, "its own USB-C serial port")
    # Below the supply's near corner, clear of its cord running off to the right.
    pp = P(50 + dx, 140 + dy, 0)
    leader((pp[0] - 2, pp[1] + 4), (pp[0] - 26, pp[1] + 36))
    tag(pp[0] - 30, pp[1] + 60, "Power supply", "end")
    sub(pp[0] - 30, pp[1] + 60 + 26 * TS, "5–12 V DC, cord to the wall", "end")
    if arm is LAYOUT["arms"][0]:
        jp = P(46 + dx, 58 + dy, 2.6)
        leader((jp[0] + 2, jp[1] + 13), (jp[0] + 8, jp[1] + 50))
        tag(jp[0] + 10, jp[1] + 74, "Barrel jack", "middle")

# The Mac needs no name; in the one-arm figure its USB-C port is tagged.
if not two:
    up = P(xr + 2, yl + 51, BH)
    leader((up[0] - 40, up[1] + 28), (up[0] - 66, up[1] + 64))
    tag(up[0] - 70, up[1] + 88, "USB-C", "end")

# ---------- CH343P inset ----------

if LAYOUT["inset"] and TS == 1:

    # The inset is its own layer (chip-labels.svg, beside the output): wiring.html places it, with the
    # chip render under it, a fixed distance in from the corner like the toggle above, and draws the
    # dashed pointer to it from the board's chip (P(11, 7.5, 2.4) in this canvas).
    main_out, out = out, []
    (ix, iy), iw, ih = (1, 1), VARIANTS["chip"]["W"], VARIANTS["chip"]["H"]
    out.append(f'<rect x="{ix}" y="{iy}" width="{iw}" height="{ih}" fill="none" stroke="{INK}" stroke-width="2"/>')
    text(ix + 30, iy + 50, "CH343P, the USB-to-serial chip", 30, weight=500)
    text(ix + 30, iy + 84, "TXD PIN 4, RXD 5, D+ 7, D− 8", 21, fill=INK2, spacing=1.8)

    # The chip itself is a render (layout.py's "chip"), drawn into this frame by wiring.html; these are
    # its labels, placed with that render's projection.
    CL = VARIANTS["chip"]

    def Pc(x, y, z=0):
        return (ix + CL["OX"] - (x - y) * C * CL["S"], iy + CL["OY"] + ((x + y) * 0.5 - z) * CL["S"])

    # The two buffers, each named with the CH343P pin it serves; labels beside the parts they name.
    s4 = Pc(5.0, 1.2, 1.15)
    text(s4[0] - 64, s4[1] + 2, "SEND", 22, "end", INK, spacing=1.8)
    text(s4[0] - 64, s4[1] + 28, "from TXD", 20, "end", INK2, "m")
    s5 = Pc(6.25, 6.6, 0.9)
    text(s5[0] + 92, s5[1] + 6, "RECEIVE", 22, "start", INK, spacing=1.8)
    text(s5[0] + 92, s5[1] + 32, "to RXD", 20, "start", INK2, "m")
    u = Pc(-1.4, 4.55)
    text(u[0] + 18, u[1] - 4, "USB-C", 24, spacing=1.8)
    text(u[0] + 18, u[1] + 24, "D+ and D−", 20, fill=INK2)
    e = Pc(10.4, 5.1)
    text(e[0] - 16, e[1] - 4, "ONE DATA LINE", 24, "end", spacing=1.8)
    text(e[0] - 16, e[1] + 22, "to the servo socket", 20, "end", INK2)

style = (f"<style>.t{{font-family:{SANS}}}.m{{font-family:{MONO}}}"
         f".halo{{paint-order:stroke;stroke:{PAPER};stroke-width:{7 * TS:.0f}px;stroke-linejoin:round}}</style>")
if LAYOUT["inset"] and TS == 1:
    cw, ch = iw + 2, ih + 2
    (Path(dst).parent / "chip-labels.svg").write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {cw} {ch}" width="{cw}" height="{ch}" aria-hidden="true">'
        f'{style}\n' + "\n".join(out) + "</svg>")
    out = main_out
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" aria-hidden="true">'
       f'{style}\n' + "\n".join(out) + "</svg>")
Path(dst).write_text(svg)
print("wrote", dst)
