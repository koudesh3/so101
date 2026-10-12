"""Blender scene for the wiring diagram, drawn in the style of the blog's 3D arm (viewer.html).

    Blender -b --python figures/wiring/build_scene.py -- out.png [follower|both] [scene.blend]

writes out-a.png and out-b.png (the two colours of each pixel and how much of the first; wiring.html
dithers them at the screen's pixels) and out.json (colours for compose.py's labels).

The parts sit in the same millimetre coordinates as gen_wiring.py, and the orthographic camera is set
so a scene point lands on the pixel gen_wiring.py's P(x, y, z) gives it, so compose.py can lay the
labels straight over the render.

Blender only measures the scene. Three flat passes are rendered (one sample, no shading): which part
each pixel shows and how its face meets the light, its normal, and its position. The look is then
worked out from those in numpy, with the viewer's own rules: two tones per part (to be mixed by its
ordered dither where the figure is shown), a white glint, a heavy ink line round each silhouette against the table, thin lines where one
part passes in front of another and along sharp creases, and a faded grid on the table.

Units: 1 Blender unit = 1 mm. Axes as in gen_wiring.py: +x runs down-left on screen, +y down-right,
+z up.
"""
import math
import random
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = Path(__file__).resolve().parent
STL = HERE / "refs/sts3215/sts3215_03a_v1.stl"
args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = args[0] if args else str(HERE / "render.png")
VARIANT = args[1] if len(args) > 1 else "follower"
BLEND = args[2] if len(args) > 2 else None
sys.path.insert(0, str(HERE))
from layout import S, VARIANTS  # noqa: E402

LAYOUT = VARIANTS[VARIANT]
WORK = Path(OUT).with_suffix("")

# gen_wiring.py's projection: P(x, y, z) = (OX - (x - y) C S, OY + ((x + y) / 2 - z) S) on a W x H canvas.
OX, OY, W, H = LAYOUT["OX"], LAYOUT["OY"], LAYOUT["W"], LAYOUT["H"]
S = LAYOUT.get("S", S)
MM = 2.9 / S      # depth thresholds below are in the bench's millimetres; this rescales them
SS = 2                       # rendered at twice the output size, then box-filtered down
HEAVY, THIN = 2.6 * SS, 1.1 * SS   # ink widths in render pixels, as the viewer's 3 and 1.1 px

# The viewer's light (viewer.html LIGHT), turned to suit this camera: tops and the faces toward the
# reader (+y) lit, the +x faces in shade.
LIGHT = Vector((-0.30, 0.42, 0.86)).normalized()
VIEW = Vector((1, 1, 1)).normalized()      # toward the camera
HALF = (LIGHT + VIEW).normalized()

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
col = scene.collection


# ---------- Colour, ported from viewer.html (light theme) ----------

def lch(L, C, Hh):
    h = math.radians(Hh)
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
        u = min(1, max(0, u))
        out.append(12.92 * u if u <= 0.0031308 else 1.055 * u ** (1 / 2.4) - 0.055)
    return out


def rel_lum(c):
    v = [x / 12.92 if x <= 0.03928 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
    return 0.2126 * v[0] + 0.7152 * v[1] + 0.0722 * v[2]


def contrast(a, b):
    x, y = rel_lum(a), rel_lum(b)
    return (max(x, y) + 0.05) / (min(x, y) + 0.05)


PAPER, INK = [1.0] * 3, [0.07] * 3
GRID = [0.5] * 3


def legible_l(L, C, Hh, m):
    for _ in range(60):
        if contrast(lch(L, C, Hh), PAPER) >= m:
            break
        L -= 1
    return L


JOINTS = [[48, 60, 262], [68, 62, 60], [46, 62, 320], [66, 45, 190], [52, 65, 28], [70, 60, 110]]


def servo_mat(j):
    L, C, Hh = JOINTS[j]
    lk = legible_l(L, C, Hh, 3.4)
    return dict(col=lch(lk, C, Hh), shade=lch(max(18, lk - 24), C * 1.05, Hh), lit=0.9, dark=0.6)


def tone(L, C=0, Hh=0, lit=0.9, dark=0.6, drop=20):
    """A part's two tones: its colour, and a darker one for the faces turned from the light."""
    return dict(col=lch(L, C, Hh), shade=lch(max(8, L - drop), C * 1.05, Hh), lit=lit, dark=dark)


TAPE = lch(legible_l(66, 36, 248, 3.05), 36, 248)
PAL = {
    # Light parts, mostly paper with a sparse dither of their colour, like the arm's printed links.
    "aluminium": tone(84, 2, 250, lit=0.34, dark=0.62, drop=20),
    "trackpad": tone(88, 2, 250, lit=0.2, dark=0.5),
    "metal": tone(80, 2, 250, lit=0.4, dark=0.62),
    "plastic": dict(col=lch(84, 10, 78), shade=lch(64, 16, 68), lit=0.34, dark=0.62),   # the arm's PRINTED
    "capacitor": tone(86, 3, 250, lit=0.3, dark=0.6),
    "data": tone(96, 0, 0, lit=0.12, dark=0.62, drop=26),
    "usb cable": tone(70, 4, 80, lit=0.7, dark=0.6),
    # Dark parts, solid.
    "keyboard": tone(24, 1, 250, lit=0.95, dark=0.6, drop=10),
    "key": tone(34, 1, 250, lit=0.95, dark=0.6, drop=14),
    "screen": tone(20, 3, 260, lit=1.0, dark=0.6, drop=8),
    "code": dict(col=TAPE, shade=TAPE, lit=1.0, dark=1.0),
    "port": tone(16, 0, 0, lit=1.0, dark=1.0),
    "overmould": tone(38, 1, 250, lit=0.95, dark=0.6, drop=16),
    "pcb": tone(24, 3, 250, lit=1.0, dark=0.6, drop=10),
    "header": tone(30, 1, 250, lit=1.0, dark=0.6, drop=12),
    "hole": dict(col=PAPER, shade=PAPER, lit=1.0, dark=1.0),
    "gold": tone(72, 45, 85, lit=0.9, dark=0.6),
    "chip": dict(col=TAPE, shade=lch(40, 38, 252), lit=1.0, dark=0.6),
    "component": tone(36, 1, 250, lit=0.95, dark=0.6, drop=14),
    "resistor": tone(60, 25, 60, lit=0.9, dark=0.6),
    "terminal": tone(legible_l(62, 48, 150, 3.05), 48, 150, lit=0.9, dark=0.6, drop=22),
    "power": servo_mat(4) | {"lit": 1.0},
    "ground": tone(20, 0, 0, lit=1.0, dark=0.6, drop=8),
    "dark plastic": tone(32, 1, 250, lit=0.95, dark=0.6, drop=14),
    # The chips: black epoxy, tinned leads, a grey laser marking.
    "ic": tone(20, 1, 250, lit=1.0, dark=0.6, drop=8),
    "lead": tone(84, 2, 250, lit=0.45, dark=0.6, drop=22),
    "marking": tone(82, 1, 250, lit=0.8, dark=0.8),
}
for j in range(6):
    PAL[f"servo {j}"] = servo_mat(j)

# Each object gets a pass index; INDEX maps it back to a palette entry and GROUP to an outline group
# (parts in one group draw no ink between them, e.g. the code on the screen).
INDEX = [None, None]      # 0: nothing, 1: the table
GROUP = [0, 1]


def tag(ob, key, group=None):
    ob.pass_index = len(INDEX)
    INDEX.append(key)
    GROUP.append(ob.pass_index if group is None else group.pass_index)
    return ob


# ---------- Shapes ----------

def link(ob, key, group=None):
    col.objects.link(ob)
    return tag(ob, key, group)


def box(x, y, z, dx, dy, dz, key, bevel=0.0, name="box", group=None):
    """An axis-aligned box from (x, y, z) to (x+dx, y+dy, z+dz), as gen_wiring.py's box()."""
    me = bpy.data.meshes.new(name)
    v = [(x + i * dx, y + j * dy, z + k * dz) for i in (0, 1) for j in (0, 1) for k in (0, 1)]
    f = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    me.from_pydata(v, [], f)
    me.update()
    ob = link(bpy.data.objects.new(name, me), key, group)
    if bevel:
        mod = ob.modifiers.new("bevel", "BEVEL")
        mod.width = bevel
        mod.segments = 4
    return ob


def cylinder(x, y, z, r, h, key, axis="z", name="cyl", segments=48):
    """A cylinder from (x, y, z) along +axis for h, radius r."""
    bpy.ops.mesh.primitive_cylinder_add(vertices=segments, radius=r, depth=h)
    ob = bpy.context.active_object
    ob.name = name
    d = {"x": Vector((1, 0, 0)), "y": Vector((0, 1, 0)), "z": Vector((0, 0, 1))}[axis]
    ob.rotation_euler = {"x": (0, math.pi / 2, 0), "y": (-math.pi / 2, 0, 0), "z": (0, 0, 0)}[axis]
    ob.location = Vector((x, y, z)) + d * h / 2
    return tag(ob, key)


def catmull(points, per=24):
    """A smooth path through the points (centripetal Catmull-Rom), as a dense list of Vectors."""
    P = [Vector(p) for p in points]
    P = [P[0] * 2 - P[1]] + P + [P[-1] * 2 - P[-2]]
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        t0 = 0.0
        t1 = t0 + max((p1 - p0).length, 1e-6) ** 0.5
        t2 = t1 + max((p2 - p1).length, 1e-6) ** 0.5
        t3 = t2 + max((p3 - p2).length, 1e-6) ** 0.5
        for s in range(per):
            t = t1 + (t2 - t1) * s / per
            a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0))
            a2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1))
            a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2))
            b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0))
            b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1))
            out.append(b1 * ((t2 - t) / (t2 - t1)) + b2 * ((t - t1) / (t2 - t1)))
    out.append(P[-2])
    return out


def tube(path, r, key, name="wire"):
    """One round conductor along a list of points."""
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = r
    cu.bevel_resolution = 6
    cu.use_fill_caps = True
    sp = cu.splines.new("POLY")
    sp.points.add(len(path) - 1)
    for p, q in zip(sp.points, path):
        p.co = (*q, 1)
    sp.use_smooth = True
    return link(bpy.data.objects.new(name, cu), key)


def ribbon(waypoints, keys, r, start_across, end_across, start_pitch=None, name="cable"):
    """Conductors bonded side by side along a smooth path through the waypoints, like a servo lead.

    start_across / end_across: the direction from the first conductor to the last where the cable
    leaves and where it arrives (the pin order in each socket). The across direction is carried
    along the path by parallel transport, and whatever turn is left over at the far end is spread
    along the length as a gentle twist, as a real lead takes it up.
    """
    path = catmull(waypoints)
    n = len(path)
    tang = [(path[min(i + 1, n - 1)] - path[max(i - 1, 0)]).normalized() for i in range(n)]
    acr = [Vector(start_across)]
    for i in range(1, n):
        a = acr[-1] - tang[i] * acr[-1].dot(tang[i])
        acr.append(a.normalized())
    want = Vector(end_across) - tang[-1] * Vector(end_across).dot(tang[-1])
    want.normalize()
    left = math.atan2(acr[-1].cross(want).dot(tang[-1]), acr[-1].dot(want))
    lengths = [0.0]
    for i in range(1, n):
        lengths.append(lengths[-1] + (path[i] - path[i - 1]).length)
    pitch = 2 * r * 1.02
    p0 = start_pitch or pitch
    m = len(keys)
    for k, key in enumerate(keys):
        pts = []
        for i in range(n):
            s = lengths[i] / lengths[-1]
            ease = s * s * (3 - 2 * s)
            a = Matrix.Rotation(left * ease, 3, tang[i]) @ acr[i]
            p_ = p0 + (pitch - p0) * min(1, s * 6)
            pts.append(path[i] + a * ((k - (m - 1) / 2) * p_))
        tube(pts, r, key, f"{name} {key}")


# ---------- The scene, section by section as in gen_wiring.py ----------

def bench():
    """The Mac, and each arm's board, supply and servos (layout.py says how many and where)."""

    SW, SD, SL = 24.8, 32.9, 45.4
    ZP = SL / 2 - 1.55
    PA, PBX = 6.3, 18.5
    X = [100 + 44 * i for i in range(6)]
    R_WIRE = 0.62
    LEAD = ["ground", "power", "data"]       # across each socket, -x to +x
    rng = random.Random(3)

    mac_before = set(bpy.data.objects)
    # MacBook Pro 14-inch, 312.6 x 221.2 mm; only its left rear corner is in frame.
    LW, LD, BH = 312.6, 221.2, 11.0
    xr, yl = -85.0, -60.0
    xl = xr - LW
    box(xl, yl, 0, LW, LD, BH, "aluminium", bevel=5, name="mac base")


    def on_top(u0, v0, u1, v1, key, h=0.15, name="top"):
        return box(xl + u0 * LW, yl + v0 * LD, BH, (u1 - u0) * LW, (v1 - v0) * LD, h, key, name=name)


    on_top(.045, .11, .955, .56, "keyboard", 0.05, "keyboard well")
    for r_ in range(6):
        v0 = .122 + r_ * .072
        for c_ in range(14):
            u0 = .053 + c_ * .0646
            on_top(u0, v0, u0 + .056, v0 + .060, "key", 0.5, "key")
    on_top(.29, .60, .71, .97, "trackpad", 0.05, "trackpad")

    tilt = math.radians(20)
    hinge = Matrix.Translation((xl, yl + 6, BH)) @ Matrix.Rotation(-tilt, 4, "X")
    lid = box(0, -4.5, 0, LW, 4.5, LD, "aluminium", bevel=3, name="lid")
    lid.data.transform(hinge)
    # The open lid would throw a shadow over half the table; like the arm's floor, the table only shows
    # the shadows of the small parts on it.
    lid.visible_shadow = False
    screen = box(0.02 * LW, -0.05, 0.04 * LD, 0.96 * LW, 0.1, 0.93 * LD, "screen", name="screen")
    screen.data.transform(hinge)
    screen.visible_shadow = False
    for i_, (u0, wd) in enumerate([(.80, .10), (.76, .16), (.76, .08), (.80, .12), (.76, .17), (.80, .07)]):
        ln = box(u0 * LW, 0.06, (.20 - i_ * .028) * LD, wd * LW, 0.05, 0.9, "code", name="code", group=screen)
        ln.data.transform(hinge)
        ln.visible_shadow = False

    TB1, TB2 = yl + 43, yl + 59
    for yc, w_, h_ in ((yl + 22, 15, 3.6), (TB1, 8.3, 2.6), (TB2, 8.3, 2.6), (yl + 74, 3.5, 3.5)):
        box(xr - 0.6, yc - w_ / 2, BH / 2 - h_ / 2, 0.8, w_, h_, "port", bevel=min(h_, w_) / 2 - 0.05, name="port")
    # The layout can slide the Mac along y, e.g. to bring its ports level with the gap between two rows.
    MAC_DY = LAYOUT.get("mac_dy", 0)
    for ob in set(bpy.data.objects) - mac_before:
        ob.location.y += MAC_DY
    TB1, TB2 = TB1 + MAC_DY, TB2 + MAC_DY

    # Servos: STS3215 on its side, back face toward the viewer (ports and idler wheel on that face).
    # The mesh is in metres: its x is the case's length (wheels toward +x), y its width, z the horn axis
    # (output horn on +z, ports and idler on -z). Here length runs up, width along the chain, and the
    # port face looks along +y.
    bpy.ops.wm.stl_import(filepath=str(STL))
    src = bpy.context.selected_objects[0]
    src.data.transform(Matrix.Scale(1000, 4))
    src.data.transform(Matrix(((0, -1, 0, 0), (0, 0, -1, 0), (1, 0, 0, 0), (0, 0, 0, 1))))
    for p in src.data.polygons:
        p.use_smooth = False
    for c_ in list(src.users_collection):
        c_.objects.unlink(src)


    def hole(hx, hy):
        cylinder(hx, hy, 1.6, 1.9, 0.06, "gold", name="pad")
        cylinder(hx, hy, 1.6, 1.15, 0.1, "hole", name="hole")


    HEADER_H = 2.8   # the two socket strips stand low on the real board


    def header(y0):
        box(7, y0, 1.6, 17.8, 2.5, HEADER_H, "header", name="header")
        for k_ in range(7):
            box(7.75 + 2.54 * k_, y0 + 0.75, 1.6 + HEADER_H, 1.0, 1.0, 0.05, "port", name="header hole")


    def plug(x0):
        box(x0 - 3.5, SD, ZP - 1.75, 7, 6, 3.5, "plastic", bevel=0.4, name="plug")


    def lead(x0, x1, sag=46.0):
        """From the plug at x0 on one servo's back face, down to the table, in a loop, and up into x1."""
        j = lambda a=1.5: rng.uniform(-a, a)
        zt = R_WIRE
        return [(x0, SD + 6, ZP), (x0, SD + 11, ZP - 0.6), (x0 + 0.5, SD + 16, ZP - 7), (x0 + 1.5, SD + 21, 4),
                (x0 + 3 + j(), SD + 28, zt), (x0 + 6 + j(), SD + sag - 6 + j(3), zt),
                ((x0 + x1) / 2 + j(), SD + sag + j(2), zt),
                (x1 - 6 + j(), SD + sag - 6 + j(3), zt), (x1 - 3 + j(), SD + 28, zt),
                (x1 - 1.5, SD + 21, 4), (x1 - 0.5, SD + 16, ZP - 7), (x1, SD + 11, ZP - 0.6), (x1, SD + 6, ZP)]


    R_USB, R_DC = 1.8, 1.3


    def arm_bus(cord="front"):
        """One arm's bus, built where the follower's sits: board, its power and supply (with a cord off to
        the wall), six servos and the lead through them. The caller moves the result into place."""
        # Board: Seeed Servo Driver Board, 42 x 25 mm, parts placed from Seeed's product photo.
        box(29, -6.5, 2.7, 7, 5.5, 3.6, "plastic", bevel=0.4, name="board plug")
        box(0, 0, 0, 42, 25, 1.6, "pcb", name="pcb")
        hole(26.5, 2.6); hole(39.4, 2.6)
        box(0, 6.6, 1.6, 7.3, 9, 3.2, "metal", bevel=0.8, name="usb receptacle")
        header(2.25)
        box(9.5, 6, 1.6, 3, 3, 0.8, "ic", bevel=0.15, name="CH343P")
        for cx_, cy_ in ((14, 10.2), (10, 12.7)):
            box(cx_, cy_, 1.6, 2.9, 1.6, 1.0, "component", name="buffer")
        for cx_, cy_ in ((15, 6.5), (17.5, 7.5), (20, 9), (22, 6.3), (18, 12)):
            box(cx_, cy_, 1.6, 1.0, 0.6, 0.4, "metal", name="passive")
        box(28.5, -1, 1.6, 8, 5.5, 6, "plastic", name="servo socket")
        for k_ in range(3):
            box(30.2 + 2.5 * k_, 4.5, 1.6, 0.6, 1.9, 2.4, "metal", name="pin")
        for ty in (6.6, 8.0):
            box(24.6, ty, 1.6, 1.5, 0.9, 0.3, "resistor", name="resistor")
        box(26.3, 10, 1.6, 4.4, 4.4, 3, "component", bevel=0.4, name="inductor")
        cylinder(37, 10, 1.6, 2.7, 5.6, "capacitor", name="capacitor")
        header(17.5)
        box(27, 15.6, 1.6, 2.5, 5, 8, "component", name="jumper")
        box(31, 13.5, 1.6, 7.5, 7, 8.5, "terminal", name="screw terminal")
        for ty in (15.25, 18.75):
            cylinder(34.75, ty, 10.1, 1.35, 0.2, "metal", name="screw")
            box(38.45, ty - 1.1, 2.4, 0.1, 2.2, 2.4, "port", name="wire entry")
        for k_ in range(8):
            cylinder(2.5 + 2.54 * k_, 22.8, 1.6, 0.8, 0.06, "gold", name="pad")
        hole(25.6, 22.2); hole(39.4, 22.4)
        # The board's end of its USB-C lead.
        box(-21.5, 4.9, 0.2, 20, 12.4, 6.5, "overmould", bevel=1.2, name="usb plug")
        box(-1.5, 7.0, 2.0, 1.5, 8.2, 2.4, "metal", name="usb tongue")

        # Power: a two-wire pigtail from the screw terminal to a barrel jack, the supply's plug and lead,
        # the supply, and its mains cord running off toward the wall.
        ribbon([(38.5, 17, 3.6), (42, 17, 3.4), (45, 20, 1.8), (46, 27, R_WIRE), (46, 37, R_WIRE), (46, 45, 2.9)],
               ["power", "ground"], R_WIRE, (0, -1, 0), (-1, 0, 0), start_pitch=3.5, name="pigtail")
        cylinder(46, 45, 2.9, 2.9, 13, "dark plastic", axis="y", name="barrel jack")
        cylinder(46, 59, 2.9, 2.2, 1.2, "metal", axis="y", name="barrel sleeve")
        cylinder(46, 60, 2.9, 2.4, 11, "dark plastic", axis="y", name="barrel plug")
        tube(catmull([(46, 71, 2.9), (46, 77, 2.2), (45, 86, R_DC), (40, 96, R_DC), (37, 105, R_DC), (38, 112, 5)]),
             R_DC, "dark plastic", "supply lead")
        box(26, 112, 0, 24, 40, 16, "dark plastic", bevel=1.5, name="power supply")
        box(35.5, 152, 5, 5, 6, 6, "dark plastic", bevel=1.0, name="cord strain relief")
        if cord == "front":
            way = [(40, 180, R_USB), (47, 215, R_USB), (43, 260, R_USB), (50, 320, R_USB), (46, 420, R_USB),
                   (52, 600, R_USB)]
        else:
            way = [(46, 176, R_USB), (70, 181, R_USB), (120, 176, R_USB), (200, 180, R_USB), (320, 175, R_USB),
                   (500, 179, R_USB), (800, 177, R_USB)]
        tube(catmull([(38, 158, 8), (38, 166, 4)] + way), R_USB, "dark plastic", "mains cord")

        for i, x in enumerate(X):
            ob = bpy.data.objects.new("servo", src.data)
            ob.location = (x + SW / 2, SD / 2, SL / 2)
            link(ob, f"servo {i}")
            plug(x + PA)
            if i < 5:
                plug(x + PBX)

        # Board to servo 1: out of the board's socket toward -y, down, round the end of the board, along
        # the table and up into servo 1's first port; then servo to servo, a loop on the table each.
        xa = X[0] + PA
        zt = R_WIRE
        ribbon([(32.5, -6.5, 4.5), (32.5, -11, 4.2), (33.5, -16, 1.5), (37, -22, zt), (46, -25, zt), (54, -17, zt),
                (57, 2, zt), (62, 30, zt), (76, SD + 42, zt), (94, SD + 44, zt), (xa - 3, SD + 28, zt),
                (xa - 1.5, SD + 21, 4), (xa - 0.5, SD + 16, ZP - 7), (xa, SD + 11, ZP - 0.6), (xa, SD + 6, ZP)],
               LEAD, R_WIRE, (1, 0, 0), (1, 0, 0), name="lead 0")
        for i in range(5):
            ribbon(lead(X[i] + PBX, X[i + 1] + PA, 44 + 3 * (i % 2)), LEAD, R_WIRE, (1, 0, 0), (1, 0, 0),
                   name=f"lead {i + 1}")


    PORTS = [TB1, TB2]
    for arm in LAYOUT["arms"]:
        before = set(bpy.data.objects)
        arm_bus(arm["cord"])
        shift = Vector((arm["dx"], arm["dy"], 0))
        for ob in set(bpy.data.objects) - before:
            ob.location += shift
        # USB-C lead from the Mac's port to this board: a plug in the port, the cable across the table.
        py = PORTS[arm["port"]]
        box(xr, py - 4.1, BH * 0.5 - 1.2, 1.6, 8.2, 2.4, "metal", name="usb tongue")
        box(xr + 1.6, py - 6.2, BH * 0.5 - 3.25, 19, 12.4, 6.5, "overmould", bevel=1.2, name="usb plug")
        bx, by = arm["dx"] - 21.5, arm["dy"] + 11.1      # the back of the board's plug
        if by > py:      # the board is in front of the port: out, and curving round onto it
            way = [(xr + 33, py + 3, R_USB + 0.4), (bx - 30, (py + by) / 2 - 2, R_USB), (bx - 21, by - 2, R_USB)]
        else:            # the board is behind: out, and back along the table to it
            way = [(xr + 33, py - 3, R_USB + 0.4), (xr + 42, py - 30, R_USB), (bx - 34, by + 40, R_USB),
                   (bx - 24, by + 4, R_USB)]
        tube(catmull([(xr + 20.6, py, BH * 0.5), (xr + 27, py + 0.5, 4.6)] + way
                     + [(bx - 10, by, 3.0), (bx, by, 3.45)]), R_USB, "usb cable", "usb cable")

def chip():
    """The inset: the CH343P close up, its leads, the traces to the two buffers and to USB, and the one
    data wire out of the buffers. Same millimetres; the inset's camera is just much closer."""
    body = box(0, 0, 0.02, 3, 3, 0.73, "ic", bevel=0.1, name="CH343P")
    for p_ in (0.75, 1.25, 1.75, 2.25):     # 16 leads, four a side, standing out of the body
        box(p_ - .13, 3, 0.02, .26, .45, .17, "lead", name="lead")
        box(p_ - .13, -.45, 0.02, .26, .45, .17, "lead", name="lead")
        box(3, p_ - .13, 0.02, .45, .26, .17, "lead", name="lead")
        box(-.45, p_ - .13, 0.02, .45, .26, .17, "lead", name="lead")
    dot = cylinder(2.5, 0.5, 0.75, 0.17, 0.01, "marking", name="pin 1")   # pin 1, at the far end of the near-left face
    GROUP[dot.pass_index] = body.pass_index        # no ink round the marks on the body
    # The laser marking, as on a real part: maker, part number, lot code, in a light speckled grey.
    try:
        mono = bpy.data.fonts.load("/System/Library/Fonts/SFNSMono.ttf")
    except RuntimeError:
        mono = None
    for text_, size, x_ in (("WCH", 0.46, 0.72), ("CH343P", 0.58, 1.42), ("2335B8L", 0.44, 2.16)):
        cu = bpy.data.curves.new("marking", "FONT")
        cu.body = text_
        cu.size = size
        if mono:
            cu.font = mono
        cu.align_x, cu.align_y = "CENTER", "CENTER"
        t = bpy.data.objects.new("marking", cu)
        t.location = (x_, 1.5, 0.752)
        t.rotation_euler = (0, 0, math.pi / 2)     # reads along +y, left to right on screen
        link(t, "marking", group=body)

    def trace(*pts, w=0.2):
        """A gold trace on the board through points, each leg along one axis: one piece, so a bend is
        a square corner (each leg runs half a width past its ends) with no ink across it."""
        first = None
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            xa, xb, ya, yb = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)
            leg = box(xa - w / 2, ya - w / 2, 0, xb - xa + w, yb - ya + w, 0.03, "gold", name="trace")
            if first is None:
                first = leg
            else:
                GROUP[leg.pass_index] = first.pass_index

    # As Seeed's schematic has it (U7 CH343P, U4 SN74LVC1G126, U5 SN74LVC1G125): pins 1-4 run along the
    # near-left face toward the front corner, 5-8 along the near-right face away from it. TXD is pin 4
    # and RXD pin 5, either side of that corner; D+ and D- are pins 7 and 8.
    def buffer(x0, y0, three_on_left, name, L, Wd, H, pitch):
        """A five-lead package: L x Wd body, three leads down one long side and two down the other."""
        box(x0, y0, 0.02, Wd, L, H, "ic", bevel=0.08, name=name)
        three, two = (x0 - 0.35, x0 + Wd) if three_on_left else (x0 + Wd, x0 - 0.35)
        for y_ in (L / 2 - pitch, L / 2, L / 2 + pitch):
            box(three, y0 + y_ - 0.12, 0.02, 0.35, 0.24, 0.12, "lead", name="lead")
        for y_ in (L / 2 - pitch, L / 2 + pitch):
            box(two, y0 + y_ - 0.12, 0.02, 0.35, 0.24, 0.12, "lead", name="lead")

    UX = 5.0
    # U4 SN74LVC1G126DBV: SOT-23-5, 2.9 x 1.6 x 1.15 mm. A in (pin 2) on the left, Y out (pin 4) right.
    buffer(UX, 1.2, True, "U4 send buffer", 2.9, 1.6, 1.15, 0.95)
    # U5 SN74LVC1G125DCK: SC-70-5, 2.0 x 1.25 x 0.9 mm. A in from the data line on the right, Y out left.
    U5Y = 5.6
    buffer(UX, U5Y, False, "U5 receive buffer", 2.0, 1.25, 0.9, 0.65)
    trace((3.4, 2.25), (4.2, 2.25), (4.2, 2.65), (UX - 0.3, 2.65))          # TXD, pin 4, into U4's A
    trace((2.25, 3.4), (2.25, U5Y + 0.35), (UX - 0.3, U5Y + 0.35))          # RXD, pin 5, from U5's Y
    # U4's output and U5's input share the one data line, which leaves toward the servo socket.
    trace((UX + 1.9, 3.6), (8.0, 3.6), (8.0, U5Y + 1.0), (UX + 1.55, U5Y + 1.0))
    trace((8.0, 5.1), (10.4, 5.1))
    # D+ and D- (pins 7 and 8), out and round toward the Mac's USB-C.
    trace((1.25, 3.4), (1.25, 4.8), (-1.4, 4.8))
    trace((0.75, 3.4), (0.75, 4.3), (-1.4, 4.3))

if LAYOUT.get("scene") == "chip":
    chip()
else:
    bench()

# The table.
me = bpy.data.meshes.new("table")
me.from_pydata([(-3000, -3000, 0), (3000, -3000, 0), (3000, 3000, 0), (-3000, 3000, 0)], [], [(0, 1, 2, 3)])
table = bpy.data.objects.new("table", me)
col.objects.link(table)
table.pass_index = 1

# ---------- Passes ----------

sun = bpy.data.lights.new("sun", "SUN")
sun.energy = math.pi
sun.angle = 0.0
for attr, value in (("shadow_filter_radius", 0.0), ("use_shadow_jitter", False)):
    if hasattr(sun, attr):
        setattr(sun, attr, value)
sun_ob = bpy.data.objects.new("sun", sun)
col.objects.link(sun_ob)
sun_ob.rotation_euler = LIGHT.to_track_quat("Z", "Y").to_euler()

world = bpy.data.worlds.new("world")
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0
scene.world = world

cam_d = bpy.data.cameras.new("iso")
cam_d.type = "ORTHO"
px_per_mm = S / math.sqrt(2 / 3)     # P draws a unit of x at C S px; isometry shows it at sqrt(2/3)
cam_d.ortho_scale = W / px_per_mm
cam_d.clip_start, cam_d.clip_end = 1, 8000
cam = bpy.data.objects.new("iso", cam_d)
col.objects.link(cam)
scene.camera = cam
C = math.cos(math.radians(30))
xmy, xpy = (OX - W / 2) / (C * S), (H / 2 - OY) / (0.5 * S)
centre = Vector(((xpy + xmy) / 2, (xpy - xmy) / 2, 0))
cam.location = centre + VIEW * 3000
cam.rotation_euler = (-VIEW).to_track_quat("-Z", "Y").to_euler()

r = scene.render
r.engine = "BLENDER_EEVEE"
r.resolution_x, r.resolution_y = W * SS, H * SS
r.film_transparent = True
r.filter_size = 0.0
r.image_settings.file_format = "OPEN_EXR"
r.image_settings.color_depth = "32"
scene.eevee.taa_render_samples = 1
for vt in ("Raw", "Standard"):
    try:
        scene.view_settings.view_transform = vt
        break
    except TypeError:
        pass


def node(nt, kind, **inputs):
    n = nt.nodes.new(kind)
    for k, v in inputs.items():
        if k == "op":
            n.operation = v
        else:
            n.inputs[k].default_value = v
    return n


def emit_material(name, build):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    em = nt.nodes.new("ShaderNodeEmission")
    nt.links.new(em.outputs[0], out.inputs[0])
    nt.links.new(build(nt), em.inputs["Color"])
    return m


def dot_with(nt, v, w):
    n = node(nt, "ShaderNodeVectorMath", op="DOT_PRODUCT")
    n.inputs[1].default_value = tuple(w)
    nt.links.new(v, n.inputs[0])
    return n.outputs["Value"]


def light_pass(nt):
    """(pass index / 1000, (n . light + 1) / 2, n . half-vector)."""
    geo = nt.nodes.new("ShaderNodeNewGeometry")
    info = nt.nodes.new("ShaderNodeObjectInfo")
    idx = node(nt, "ShaderNodeMath", op="DIVIDE")
    idx.inputs[1].default_value = 1000
    nt.links.new(info.outputs["Object Index"], idx.inputs[0])
    dd = node(nt, "ShaderNodeMath", op="MULTIPLY_ADD")
    dd.inputs[1].default_value = dd.inputs[2].default_value = 0.5
    nt.links.new(dot_with(nt, geo.outputs["Normal"], LIGHT), dd.inputs[0])
    c = nt.nodes.new("ShaderNodeCombineXYZ")
    nt.links.new(idx.outputs[0], c.inputs[0])
    nt.links.new(dd.outputs[0], c.inputs[1])
    nt.links.new(dot_with(nt, geo.outputs["Normal"], HALF), c.inputs[2])
    return c.outputs[0]


def table_light(nt):
    """The table: (1 / 1000, lit by the sun (0 in shadow), 0)."""
    diff = nt.nodes.new("ShaderNodeBsdfDiffuse")
    diff.inputs["Color"].default_value = (1, 1, 1, 1)
    rgb = nt.nodes.new("ShaderNodeShaderToRGB")
    nt.links.new(diff.outputs[0], rgb.inputs[0])
    bw = nt.nodes.new("ShaderNodeRGBToBW")
    nt.links.new(rgb.outputs[0], bw.inputs[0])
    c = nt.nodes.new("ShaderNodeCombineXYZ")
    c.inputs[0].default_value = 0.001
    nt.links.new(bw.outputs[0], c.inputs[1])
    return c.outputs[0]


def normal_pass(nt):
    geo = nt.nodes.new("ShaderNodeNewGeometry")
    n = node(nt, "ShaderNodeVectorMath", op="MULTIPLY_ADD")
    nt.links.new(geo.outputs["Normal"], n.inputs[0])
    n.inputs[1].default_value = n.inputs[2].default_value = (0.5, 0.5, 0.5)
    return n.outputs[0]


def depth_pass(nt):
    """Distance along the view, split into a coarse and a fine channel: EEVEE renders at half
    precision, which alone would round depths to a few millimetres. e = 5000 - p . view (mm), sent as
    (e / 16384, fract(e / 32), 0)."""
    geo = nt.nodes.new("ShaderNodeNewGeometry")
    t = dot_with(nt, geo.outputs["Position"], VIEW)
    e = node(nt, "ShaderNodeMath", op="MULTIPLY_ADD")
    e.inputs[1].default_value = -1.0
    e.inputs[2].default_value = 5000.0
    nt.links.new(t, e.inputs[0])
    coarse = node(nt, "ShaderNodeMath", op="DIVIDE")
    coarse.inputs[1].default_value = 16384
    nt.links.new(e.outputs[0], coarse.inputs[0])
    per = node(nt, "ShaderNodeMath", op="DIVIDE")
    per.inputs[1].default_value = 32
    nt.links.new(e.outputs[0], per.inputs[0])
    fine = node(nt, "ShaderNodeMath", op="FRACT")
    nt.links.new(per.outputs[0], fine.inputs[0])
    c = nt.nodes.new("ShaderNodeCombineXYZ")
    nt.links.new(coarse.outputs[0], c.inputs[0])
    nt.links.new(fine.outputs[0], c.inputs[1])
    return c.outputs[0]


objs = [o for o in scene.objects if o.type in ("MESH", "CURVE", "FONT")]


def render_pass(name, part_mat, table_mat):
    for o in objs:
        mat = table_mat if o.pass_index == 1 else part_mat
        if o.data.materials:
            o.data.materials[0] = mat
        else:
            o.data.materials.append(mat)
    path = f"{WORK}-{name}.exr"
    r.filepath = path
    bpy.ops.render.render(write_still=True)
    img = bpy.data.images.load(path)
    img.colorspace_settings.name = "Non-Color"
    a = np.empty(img.size[0] * img.size[1] * 4, np.float32)
    img.pixels.foreach_get(a)
    return a.reshape(img.size[1], img.size[0], 4)


L_ = render_pass("light", emit_material("light", light_pass), emit_material("table light", table_light))
nm = emit_material("normal", normal_pass)
N_ = render_pass("normal", nm, nm)
dm = emit_material("depth", depth_pass)
D_ = render_pass("depth", dm, dm)
if BLEND:
    bpy.ops.wm.save_as_mainfile(filepath=BLEND)

# ---------- The look, from the passes (rows run bottom to top, as gl_FragCoord) ----------

h, w = L_.shape[:2]
alpha = L_[..., 3] > 0.5
idx = np.where(alpha, np.rint(L_[..., 0] * 1000), 0).astype(np.int32)
d = L_[..., 1] * 2 - 1
spec = L_[..., 2]
nrm = N_[..., :3] * 2 - 1
approx = D_[..., 0] * 16384
fine = D_[..., 1] * 32
depth = fine + 32 * np.round((approx - fine) / 32)     # larger is farther from the camera
depth[~alpha] = np.inf

yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
# Where each pixel's ray meets the table (z = 0), straight from P(): exact, unlike a rendered pass.
u, v = (xx + 0.5) / SS, (h - yy - 0.5) / SS
xmy_t, xpy_t = (OX - u) / (C * S), 2 * (v - OY) / S
tx, ty = (xpy_t + xmy_t) / 2, (xpy_t - xmy_t) / 2


n_ids = len(INDEX)
col_t = np.ones((n_ids, 3), np.float32); shade_t = np.ones((n_ids, 3), np.float32)
lit_t = np.ones(n_ids, np.float32); dark_t = np.ones(n_ids, np.float32)
for i, key in enumerate(INDEX):
    if key is not None:
        p = PAL[key]
        col_t[i], shade_t[i], lit_t[i], dark_t[i] = p["col"], p["shade"], p["lit"], p["dark"]
group_t = np.array(GROUP, np.int32)

part = idx >= 2
base, shd = col_t[idx], shade_t[idx]
lit, dark = lit_t[idx], dark_t[idx]
lit_face = d > 0.12
glint = lit_face & (spec > 0.6 ** (1 / 48))
# Each pixel as the viewer's dither sees it: two colours and how much of the first. A lit face is its
# colour over paper at its "lit" coverage; a face in shade, its shade colour over its colour at "dark".
# The dither itself is applied where the figure is shown, at the screen's own pixels (wiring.html).
c1 = np.where(lit_face[..., None], base, shd)
c2 = np.where(lit_face[..., None], np.float32(1.0), base)
cov = np.where(lit_face, lit, dark)
c1[glint] = c2[glint] = 1.0
paper = ~part
c1[paper] = GRID
c2[paper] = 1.0
cov = np.where(part, cov, 0.0).astype(np.float32)

# The table: white, with a grid of 25 mm and 100 mm lines fading with distance (no cast shadows).
tbl = idx == 1


def grid_cov(pitch, half):
    gx = np.abs((tx / pitch - 0.5) % 1.0 - 0.5) * pitch
    gy = np.abs((ty / pitch - 0.5) % 1.0 - 0.5) * pitch
    return (np.minimum(gx, gy) < half).astype(np.float32)


fade = 1 - np.clip((np.hypot(tx - 170, ty - 50) - 150) / 230, 0, 1)
fade = fade * fade * (3 - 2 * fade)
gcov = np.maximum(grid_cov(25, 0.22) * 0.42, grid_cov(100, 0.3) * 0.85) * fade
if LAYOUT.get("grid", True):
    cov[tbl] = gcov[tbl]

# Ink. Pixels near a part but on the table: the heavy silhouette. Pixels of a part with a nearer
# part (another outline group, or the same one well in front) close by: thin. Sharp creases: thin.
grp = group_t[idx]


def disc(radius):
    rr = int(math.ceil(radius))
    return [(dy, dx) for dy in range(-rr, rr + 1) for dx in range(-rr, rr + 1) if dy * dy + dx * dx <= radius * radius]


def shifted(a, dy, dx, fill):
    """a moved by (dy, dx): out[y, x] = a[y - dy, x - dx]."""
    o = np.full_like(a, fill)
    H_, W_ = a.shape[:2]
    o[max(dy, 0):H_ + min(dy, 0), max(dx, 0):W_ + min(dx, 0)] = a[max(-dy, 0):H_ - max(dy, 0), max(-dx, 0):W_ - max(dx, 0)]
    return o


heavy = np.zeros((h, w), bool)
for dy, dx in disc(HEAVY):
    heavy |= shifted(part, dy, dx, False)
heavy &= ~part

thin = np.zeros((h, w), bool)
for dy, dx in disc(THIN):
    g2 = shifted(grp, dy, dx, 0)
    z2 = shifted(depth, dy, dx, np.inf)
    thin |= part & (g2 >= 2) & (((g2 != grp) & (z2 < depth - 0.3 * MM)) | (z2 < depth - 2.5 * MM))

crease = np.zeros((h, w), bool)
for dy, dx in ((0, 1), (1, 0)):
    n2 = shifted(nrm, dy, dx, 0.0)
    same = shifted(idx, dy, dx, 0) == idx
    edge = part & same & (np.sum(nrm * n2, axis=-1) < math.cos(math.radians(32)))
    crease |= edge | shifted(edge, -dy, -dx, False)
ink = heavy | thin | crease
c1[ink] = c2[ink] = INK
cov[ink] = 1.0

# Down to the output size, a box filter over each SS x SS block, and out as two images: the first
# colour with its coverage as alpha (OUT-a.png), and the second colour (OUT-b.png).
def small(a):
    return a.reshape(h // SS, SS, w // SS, SS, *a.shape[2:]).mean(axis=(1, 3))


def save(rgb, alpha, path):
    im = bpy.data.images.new(Path(path).name, w // SS, h // SS, alpha=True)
    im.alpha_mode = "STRAIGHT"
    im.pixels.foreach_set(np.concatenate([rgb, alpha[..., None]], axis=-1).astype(np.float32).ravel())
    im.filepath_raw = path
    im.file_format = "PNG"
    im.save()


stem = str(Path(OUT).with_suffix(""))
save(small(c1), small(cov), stem + "-a.png")
save(small(c2), np.ones((h // SS, w // SS), np.float32), stem + "-b.png")
# The colours the labels use, so compose.py's chips and wire swatches match the render.
import json
hexc = lambda c: "#%02x%02x%02x" % tuple(round(min(1, max(0, v)) * 255) for v in c)
Path(OUT).with_suffix(".json").write_text(json.dumps({
    "servos": [hexc(PAL[f"servo {j}"]["col"]) for j in range(6)],
    "power": hexc(PAL["power"]["col"]), "ground": hexc(PAL["ground"]["col"]), "data": hexc(PAL["data"]["col"]),
    "chip": hexc(PAL["chip"]["col"]),
}))
print("rendered", OUT)
