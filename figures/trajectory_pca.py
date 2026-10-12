"""HTML figure: one episode's actions and observations as two paths through joint space.

Each tick of a teleop replay (the events `so101 replay` writes, or the dashboard streams) has the
goal positions sent to the robot (G: the action) and the robot's own joint values (F: the
observation). The five arm joints are put in the post's coordinates, each mapped to [0, 1] across
its range in the follower's URDF, and the gripper's 0 to 100% of its range to [0, 1] as well. The
two directions they vary along most are found (PCA over both paths together) and
both are projected onto them, so the observations show as a second curve trailing the actions.

    python3 figures/trajectory_pca.py grab1.events.jsonl > trajectory.html
"""
import gzip
import json
import math
import sys
from pathlib import Path

MODEL = Path(__file__).resolve().parent.parent / "src/report/assets/model.json"
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

STEP = 2   # keep every 2nd tick: 50 points a second, plenty for a smooth curve


def load(path):
    opener = gzip.open if path.endswith(".gz") else open
    hz, actions, observations = 100.0, [], []
    with opener(path, "rt") as f:
        for line in f:
            e = json.loads(line)
            if e.get("k") == "hello":
                hz = e.get("hz", hz)
            elif e.get("k") == "t" and e.get("G") and e.get("F"):
                actions.append(e["G"])
                observations.append(e["F"])
    return hz, actions[::STEP], observations[::STEP]


def unit(rows):
    """Arm joints in degrees → [0, 1] across each joint's URDF range; the gripper's percent → [0, 1]."""
    joints = {j["name"]: j for j in json.loads(MODEL.read_text())["arms"]["follower"]["joints"]}
    span = [(math.degrees(joints[n]["lower"]), math.degrees(joints[n]["upper"])) for n in ARM]
    return [[(r[j] - lo) / (hi - lo) for j, (lo, hi) in enumerate(span)] + [r[5] / 100] for r in rows]


def pca(rows, k=2):
    n, d = len(rows), len(rows[0])
    mean = [sum(r[j] for r in rows) / n for j in range(d)]
    z = [[r[j] - mean[j] for j in range(d)] for r in rows]
    cov = [[sum(r[a] * r[b] for r in z) / n for b in range(d)] for a in range(d)]

    # Power iteration, each component kept orthogonal to the ones before it.
    def top(m, found):
        v = [1.0 + 0.1 * j for j in range(d)]
        for _ in range(1000):
            w = [sum(m[a][b] * v[b] for b in range(d)) for a in range(d)]
            for u in found:
                dot = sum(w[a] * u[a] for a in range(d))
                w = [w[a] - dot * u[a] for a in range(d)]
            norm = math.sqrt(sum(x * x for x in w))
            v = [x / norm for x in w]
        lam = sum(v[a] * sum(m[a][b] * v[b] for b in range(d)) for a in range(d))
        return v, lam

    total = sum(cov[j][j] for j in range(d))
    vecs, shares = [], []
    for _ in range(k):
        v, lam = top(cov, vecs)
        vecs.append(v); shares.append(lam / total)
    return mean, vecs, shares


def main(path):
    hz, actions, observations = load(path)
    ua, uo = unit(actions), unit(observations)
    mean, vecs, shares = pca(ua + uo, 2)

    def project(r):
        z = [r[j] - mean[j] for j in range(len(r))]
        return [round(sum(z[j] * v[j] for j in range(len(r))), 4) for v in vecs]

    data = {
        "dt": STEP / hz,
        "action": [project(r) for r in ua],
        "observation": [project(r) for r in uo],
        # The observations as recorded, to pose a small 3D robot alongside (five joints in degrees,
        # the gripper as a percent of its range).
        "pose": [[round(v, 1) for v in r] for r in observations],
        "explained": [round(f, 3) for f in shares],
    }
    template = open(__file__.replace("trajectory_pca.py", "trajectory_template.html")).read()
    sys.stdout.write(template.replace("/*DATA*/null", json.dumps(data, separators=(",", ":"))))


if __name__ == "__main__":
    main(sys.argv[1])
