"""A synthetic teleop episode: from a start pose the gripper tip moves in a straight line, around a
tilted circle, around a rectangle standing in a vertical plane, and back to exactly where it began,
opening and closing the gripper three times along the way.

Writes events in the format `so101 replay` produces (one "t" line per tick with G, the goal sent,
and F, the follower's position), so figures/trajectory_pca.py can draw it.

- Forward kinematics is the follower URDF chain from src/report/assets/model.json, walked exactly
  as the viewer's armPose does: each child frame = parent · T(xyz, rpy) · Rz(q).
- The actions G are inverse kinematics for the tip (the URDF's gripper_frame point): damped least
  squares over shoulder pan, shoulder lift, elbow flex and wrist flex, each tick starting from the
  last solution so the joints move smoothly. Wrist roll and the gripper stay put.
- The observations F are a first-order servo closing on each goal with a 60 ms time constant,
  about the lag measured on the real arm.

    python3 figures/synthetic_episode.py > episode.events.jsonl
"""
import json
import math
import sys
from pathlib import Path

MODEL = Path(__file__).resolve().parent.parent / "src/report/assets/model.json"
HZ = 100
RADIUS = 0.05                      # metres: a circle 10 cm across
TILT = math.radians(35)            # out of the horizontal, so the circle is 3D
TAU = 0.06                         # follower time constant (s)
START = [8.23, 1.85, 12.65, 39.26]  # degrees: pan, lift, elbow, wrist flex; a comfortable pose
WRIST_ROLL, GRIPPER = 0.0, 2.0     # degrees, percent: wrist held fixed; the gripper rests closed

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


# ── 4×4 transforms ──
def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def trans(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def rx(t):
    c, s = math.cos(t), math.sin(t)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def ry(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def rz(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def rpy_t(xyz, rpy):
    return mul(trans(*xyz), mul(rz(rpy[2]), mul(ry(rpy[1]), rx(rpy[0]))))


arm = json.loads(MODEL.read_text())["arms"]["follower"]
by_child = {j["child"]: j for j in arm["joints"]}
limits = {j["name"]: (math.degrees(j["lower"]), math.degrees(j["upper"])) for j in arm["joints"] if j["type"] == "revolute"}
# The chain from the base to the tip, in order.
chain, link = [], "gripper_frame_link"
while link != "base_link":
    j = by_child[link]
    chain.insert(0, j)
    link = j["parent"]


def tip(deg):
    """Tip position (base frame, metres) for pan, lift, elbow, wrist flex in degrees."""
    q = dict(zip(JOINTS, list(deg) + [WRIST_ROLL]))
    m = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for j in chain:
        angle = math.radians(q.get(j["name"], 0.0)) if j["type"] == "revolute" else 0.0
        m = mul(mul(m, rpy_t(j["xyz"], j["rpy"])), rz(angle))
    return [m[0][3], m[1][3], m[2][3]]


def solve(target, deg, iters=40):
    """Damped least squares from deg toward the joints that put the tip at target."""
    deg = list(deg)
    for _ in range(iters):
        p = tip(deg)
        e = [target[a] - p[a] for a in range(3)]
        if math.hypot(*e) < 1e-5:
            break
        # Numeric Jacobian, metres per degree.
        J = []
        for k in range(4):
            d = list(deg); d[k] += 0.01
            pk = tip(d)
            J.append([(pk[a] - p[a]) / 0.01 for a in range(3)])
        # Δ = Jᵀ (J Jᵀ + λ² I)⁻¹ e, with J as 3×4.
        lam = 0.002
        A = [[sum(J[k][r] * J[k][c] for k in range(4)) + (lam * lam if r == c else 0) for c in range(3)] for r in range(3)]
        y = solve3(A, e)
        for k in range(4):
            lo, hi = limits[JOINTS[k]]
            deg[k] = min(hi, max(lo, deg[k] + sum(J[k][a] * y[a] for a in range(3))))
    return deg


def solve3(A, b):
    """Solve a 3×3 linear system (Cramer's rule)."""
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det(A)
    out = []
    for c in range(3):
        m = [row[:] for row in A]
        for r in range(3):
            m[r][c] = b[r]
        out.append(det(m) / d)
    return out


def min_jerk(u):
    """0 → 1 with zero velocity and acceleration at both ends."""
    u = min(1.0, max(0.0, u))
    return u * u * u * (10 - 15 * u + 6 * u * u)


def main():
    p0 = tip(START)
    add = lambda a, b: [a[k] + b[k] for k in range(3)]
    lerp = lambda a, b, u: [a[k] + (b[k] - a[k]) * u for k in range(3)]

    # Keyframes, relative to where the tip starts. x is out from the base, y sideways, z up.
    line_end = add(p0, [0.02, 0.06, 0.06])
    u = [1.0, 0.0, 0.0]                                  # circle plane: x, and y tilted up toward z
    v = [0.0, math.cos(TILT), math.sin(TILT)]
    circle_centre = [line_end[k] - RADIUS * u[k] for k in range(3)]
    rect_c = add(p0, [-0.01, -0.07, 0.05])              # a rectangle standing in the y–z plane
    w, h = 0.10, 0.07
    rect = [add(rect_c, [0, -w / 2, -h / 2]), add(rect_c, [0, w / 2, -h / 2]),
            add(rect_c, [0, w / 2, h / 2]), add(rect_c, [0, -w / 2, h / 2])]

    # The path as segments, each a function of u in [0, 1] and a duration.
    def line(a, b, speed=0.08):
        return (lambda s: lerp(a, b, min_jerk(s))), max(0.6, math.dist(a, b) / speed)

    def hold(p, t=0.4):
        return (lambda s: p), t

    # A pause in which the gripper opens wide and closes back to where it was. The gripper does not
    # move the tip point, so it is scheduled on its own.
    GRIP_OPEN = 85.0

    def grip_hold(p, t=1.2):
        f, _ = hold(p, t)
        return f, t, lambda s: GRIPPER + (GRIP_OPEN - GRIPPER) * math.sin(math.pi * s) ** 2

    def circle():
        return (lambda s: [circle_centre[k] + RADIUS * (math.cos(2 * math.pi * min_jerk(s)) * u[k] + math.sin(2 * math.pi * min_jerk(s)) * v[k]) for k in range(3)]), 4.0

    segments = [hold(p0), line(p0, line_end), grip_hold(line_end), circle(), hold(line_end), line(line_end, rect[0]), grip_hold(rect[0])]
    for a, b in zip(rect, rect[1:] + rect[:1]):
        segments += [line(a, b, 0.06), hold(b, 0.25)]
    segments += [line(rect[0], p0), grip_hold(p0), hold(p0, 0.6)]   # a last still moment, for the follower to settle
    points, grips = [], []
    for seg in segments:
        f, dur = seg[0], seg[1]
        g = seg[2] if len(seg) > 2 else (lambda s: GRIPPER)
        n = max(1, round(dur * HZ))
        points += [f(i / n) for i in range(1, n + 1)]
        grips += [g(i / n) for i in range(1, n + 1)]

    out = sys.stdout
    out.write(json.dumps({"k": "hello", "hz": float(HZ), "joints": JOINTS + ["gripper"], "synthetic": True}) + "\n")
    deg = solve(points[0], START, iters=200)
    # The arm has a joint to spare for a tip position, so the IK can come home with the tip in place
    # but the joints a little off; the last half second blends them back to the exact start.
    settle = int(0.5 * HZ)
    follower = list(START) + [GRIPPER]
    worst = 0.0
    alpha = 1 - math.exp(-1 / (HZ * TAU))
    for i, p in enumerate(points):
        deg = solve(p, deg)
        worst = max(worst, math.dist(tip(deg), p))
        if i >= len(points) - settle:
            k = (i - (len(points) - settle) + 1) / settle
            deg = [d + (s0 - d) * min_jerk(k) for d, s0 in zip(deg, START)]
        goal = deg + [grips[i]]
        follower = [f + (g - f) * alpha for f, g in zip(follower, goal)]
        G = [round(x, 2) for x in deg] + [WRIST_ROLL, round(grips[i], 2)]
        F = [round(x, 2) for x in follower[:4]] + [WRIST_ROLL, round(follower[4], 2)]
        out.write(json.dumps({"k": "t", "t": int(i * 1e6 / HZ), "G": G, "F": F}) + "\n")
    print(f"{len(points) / HZ:.1f} s, worst tip error {worst * 1000:.2f} mm, end joints {deg} vs start {START}", file=sys.stderr)


if __name__ == "__main__":
    main()
