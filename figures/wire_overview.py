"""SVG: the teleop loop, the post's wire-overview.svg, drawn like its other figures (the bus figure's
servo boxes: a coloured chip and an uppercase name in the corner, grey field labels, mono values;
the arm viewer's letter-spaced tags). Each box's chip is the colour of the arrow it sends.

    python3 figures/wire_overview.py > wire-overview.svg                  # across the page
    python3 figures/wire_overview.py --narrow > wire-overview-narrow.svg  # stacked, for a phone

The animation runs one tick slowed down: the leader sends, the host holds, the host writes the
follower, the follower holds, the follower answers; a box tints the instant a dot leaves or reaches it.
"""
import sys

NARROW = "--narrow" in sys.argv[1:]
INK, INK2, INK3, RULE = "#121212", "#555555", "#737373", "#d9d9d9"
BLUE, ORANGE, TEAL = "#007cd7", "#cd7127", "#009a93"
SANS = "-apple-system, BlinkMacSystemFont, 'SF Pro Text', 'Inter', 'Helvetica Neue', Arial, sans-serif"
MONO = "'SF Mono', ui-monospace, 'JetBrains Mono', Menlo, Consolas, monospace"
T = 4.2  # one loop of the animation, s

BOXES = {  # name: tag, chip letter, chip colour, fields (label, value)
    "leader": ("Leader arm", "L", BLUE, [("torque", "off"), ("moved by", "operator")]),
    "host": ("Host", "H", ORANGE, [("ports", "2 × USB-C"), ("rate", "100 Hz")]),
    "follower": ("Follower arm", "F", TEAL, [("torque", "on"), ("moves to", "its goal")]),
}

if NARROW:
    W, H = 360, 384
    BW, BH = 320, 76
    at = {"leader": (20, 8), "host": (20, 154), "follower": (20, 300)}
    CX, LX, RX = 180, 150, 210
    bottom = lambda k: at[k][1] + BH
    LEGS = [  # from, to, colour, path, start s, end s, tag lines, tag x, anchor, first tag y
        ("leader", "host", BLUE, f"M{CX} {bottom('leader')}V{at['host'][1]}", 0.0, 0.8,
         ["LEADER POSITIONS", "EACH TICK"], CX + 12, "start", bottom("leader") + 30),
        ("host", "follower", ORANGE, f"M{LX} {bottom('host')}V{at['follower'][1]}", 1.4, 2.2,
         ["GOAL, ONE", "CAPPED STEP"], LX - 12, "end", bottom("host") + 30),
        ("follower", "host", TEAL, f"M{RX} {at['follower'][1]}V{bottom('host')}", 2.8, 3.6,
         ["FOLLOWER", "POSITIONS"], RX + 12, "start", bottom("host") + 30),
    ]
    LINKS = []
else:
    W, H = 880, 226
    BW, BH = 170, 96
    at = {"leader": (10, 60), "host": (355, 60), "follower": (700, 60)}
    LEGS = [
        ("leader", "host", BLUE, "M95 60V24H420V60", 0.0, 0.8, ["LEADER POSITIONS, EACH TICK"], 257, "middle", 16),
        ("host", "follower", ORANGE, "M460 60V24H785V60", 1.4, 2.2, ["GOAL, ONE CAPPED STEP TOWARD THE LEADER"], 622, "middle", 16),
        ("follower", "host", TEAL, "M785 156V196H440V156", 2.8, 3.6, ["FOLLOWER POSITIONS, EACH TICK"], 612, "middle", 216),
    ]
    LINKS = [(180, 355), (525, 700)]  # the USB-C leads, at mid-height

out = []


def text(x, y, s, size, fill, anchor="start", cls="t", weight=400, spacing=None):
    sp = f' letter-spacing="{spacing}"' if spacing is not None else ""
    out.append(f'<text class="{cls}" x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" '
               f'font-weight="{weight}" fill="{fill}"{sp}>{s}</text>')


def flash(box, colour, t, decay=0.45):
    x, y = at[box]
    a, b = t / T, (t + decay) / T
    kt, v = (f"0;{b:.3f};1", "0.16;0;0") if t == 0 else (f"0;{a:.3f};{a:.3f};{b:.3f};1", "0;0;0.16;0;0")
    return (f'<rect x="{x}" y="{y}" width="{BW}" height="{BH}" fill="{colour}" fill-opacity="0">'
            f'<animate attributeName="fill-opacity" dur="{T}s" repeatCount="indefinite" keyTimes="{kt}" values="{v}"/></rect>')


def dot(path, colour, t0, t1):
    a, b = t0 / T, t1 / T
    ok, ov = (f"0;{a:.3f};{a:.3f};{b:.3f};{b:.3f};1", "0;0;1;1;0;0") if t0 else (f"0;{b:.3f};{b:.3f};1", "1;1;0;0")
    return (f'<circle r="4.5" fill="{colour}" opacity="0">'
            f'<animateMotion dur="{T}s" repeatCount="indefinite" path="{path}" calcMode="linear" keyPoints="0;0;1;1" keyTimes="0;{a:.3f};{b:.3f};1"/>'
            f'<animate attributeName="opacity" dur="{T}s" repeatCount="indefinite" keyTimes="{ok}" values="{ov}"/></circle>')


markers = "".join(
    f'<marker id="w-{i}" viewBox="0 0 8 8" refX="7.5" refY="4" markerWidth="8" markerHeight="8" markerUnits="userSpaceOnUse" '
    f'orient="auto-start-reverse"><path d="M0 0.5L8 4L0 7.5z" fill="{c}"/></marker>'
    for i, c in enumerate((BLUE, ORANGE, TEAL)))
out.append(f"<defs>{markers}</defs>")
out.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
out.append('<g class="anim">' + "".join(flash(s, c, t0) + flash(d, c, t1) for s, d, c, _, t0, t1, *_ in LEGS) + "</g>")

# The USB-C leads between the boxes (across the page only; stacked, the arrows are the links).
for x1, x2 in LINKS:
    y = 60 + BH / 2
    out.append(f'<path d="M{x1} {y}H{x2}" stroke="{INK}" stroke-width="1"/>')
    text((x1 + x2) / 2, y - 7, "USB-C", 9.5, INK3, "middle", spacing=1.2)

# Boxes: the chip and name in the corner, then label / value rows.
for name, (tag, letter, colour, fields) in BOXES.items():
    x, y = at[name]
    out.append(f'<rect x="{x}" y="{y}" width="{BW}" height="{BH}" fill="none" stroke="{INK}" stroke-width="1"/>')
    out.append(f'<rect x="{x + 10}" y="{y + 10}" width="16" height="16" fill="{colour}"/>')
    text(x + 18, y + 22, letter, 11, "#fff", "middle", "m", 500)
    text(x + 34, y + 22.5, tag.upper(), 11, INK, spacing=1.3)
    row0, step = (y + 47, 19) if NARROW else (y + 58, 22)
    for i, (label, value) in enumerate(fields):
        text(x + 10, row0 + i * step, label, 11.5, INK3)
        text(x + BW - 10, row0 + i * step, value, 12, INK, "end", "m")

# Arrows and their tags.
for i, (_, _, c, path, _, _, lines, tx, anchor, ty) in enumerate(LEGS):
    out.append(f'<path d="{path}" fill="none" stroke="{c}" stroke-width="1.5" marker-end="url(#w-{i})"/>')
    for j, line in enumerate(lines):
        text(tx, ty + j * 15, line, 10.5, c, anchor, spacing=1.2)
out.append('<g class="anim">' + "".join(dot(p, c, t0, t1) for _, _, c, p, t0, t1, *_ in LEGS) + "</g>")

label = ("Data flow: each tick the teleoperation program reads present positions from both arms and writes "
         "the follower a goal one capped step toward the leader, each arm on its own USB-C port")
print(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" aria-label="{label}">'
      f"<style>.t{{font-family:{SANS}}}.m{{font-family:{MONO}}}"
      "@media (prefers-reduced-motion: reduce){.anim{display:none}}</style>")
print("\n".join(out))
print("</svg>")
