# /// script
# requires-python = ">=3.11"
# dependencies = ["yourdfpy", "trimesh", "python-fcl", "numpy", "lxml"]
# ///
"""How close the wrist camera comes to the arm, over a grid of elbow, wrist flex and wrist roll.

Writes data/camera_clearance.json, which ServoArm uses to stop the follower before the camera hits
its own upper arm or forearm. The geometry is The Robot Studio's SO-101 model with their Hex-Nut
wrist camera mount (Apache 2.0), downloaded at a pinned commit into target/. Joint angles are our calibrated
degrees: both put zero at the middle of each joint's range, and three poses checked on the real arm
(2026-11-23) needed no sign flips.

Only these three joints move the camera relative to the upper arm and forearm, so shoulder pan and
lift are held at zero. Contact with the shoulder or base, which depends on the lift too, isn't
covered.

    uv run tools/camera_clearance.py            # writes data/camera_clearance.json
"""

import http.client
import json
import multiprocessing
import re
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
import trimesh
import yourdfpy

REPO = "https://github.com/TheRobotStudio/SO-ARM100"
COMMIT = "141aea7bcac2744d92998e30268c413202a56f96"
URDF = "Simulation/SO101/so101_new_calib_camera.urdf"
ROOT = Path(__file__).resolve().parent.parent
CACHE = ROOT / "target" / "so-arm100" / COMMIT
OUT = ROOT / "data" / "camera_clearance.json"

# The grid, in degrees: a little past each joint's calibrated range.
AXES = {"elbow_flex": (-100, 100, 5), "wrist_flex": (-105, 105, 5), "wrist_roll": (-160, 165, 5)}
CAMERA = ["wrist_camera_mount_link", "wrist_camera_link"]
ARM = ["upper_arm_link", "lower_arm_link", "wrist_link"]
# The mount bolts on beside the wrist and moves with it; their hulls overlap by design.
SKIP = {("wrist_camera_mount_link", "wrist_link")}
PAIRS = [(c, a) for c in CAMERA for a in ARM if (c, a) not in SKIP]
# Hulls never overestimate clearance. Below this, the exact meshes decide.
EXACT_BELOW_MM = 30.0


def fetch() -> Path:
    """The URDF and the meshes of the links we check, downloaded once at the pinned commit."""
    raw = f"https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/{COMMIT}/"

    def get(path: str) -> Path:
        local = CACHE / path
        if local.exists():
            return local
        local.parent.mkdir(parents=True, exist_ok=True)
        for attempt in range(5):
            try:
                with urllib.request.urlopen(raw + path) as r:
                    data = r.read()
                break
            except (http.client.IncompleteRead, urllib.error.URLError):
                if attempt == 4:
                    raise
                time.sleep(1)
        partial = local.with_suffix(local.suffix + ".part")
        partial.write_bytes(data)
        partial.rename(local)
        return local

    urdf = get(URDF)
    meshes = re.findall(r'<mesh filename="([^"]+)"', urdf.read_text())
    for m in set(meshes):
        get(str(Path(URDF).parent / m))
    return urdf


def load():
    robot = yourdfpy.URDF.load(str(fetch()), build_collision_scene_graph=True, load_collision_meshes=True)
    meshes = {}
    for name in CAMERA + ARM:
        parts = []
        for c in robot.link_map[name].collisions:
            g = c.geometry.mesh
            m = trimesh.load(str(fetch().parent / g.filename), force="mesh")
            if g.scale is not None:
                m.apply_scale(g.scale)
            m.apply_transform(c.origin if c.origin is not None else np.eye(4))
            parts.append(m)
        meshes[name] = trimesh.util.concatenate(parts)
    return robot, meshes


def managers(meshes, hull: bool):
    out = {}
    for name, mesh in meshes.items():
        out[name] = trimesh.collision.CollisionManager()
        out[name].add_object(name, mesh.convex_hull if hull else mesh)
    return out


_state = {}


def _init():
    robot, meshes = load()
    _state.update(robot=robot, hull=managers(meshes, True), exact=managers(meshes, False))


def clearance(angles):
    """Millimetres from the camera to the arm at (elbow, wrist flex, wrist roll) in degrees; 0 = touching."""
    robot = _state["robot"]
    robot.update_cfg({j: np.radians(a) for j, a in zip(AXES, angles)})
    transforms = {n: robot.get_transform(n) for n in CAMERA + ARM}

    def nearest(mgr):
        for n, t in transforms.items():
            mgr[n].set_transform(n, t)
        return min(mgr[c].min_distance_other(mgr[a]) for c, a in PAIRS) * 1000

    mm = nearest(_state["hull"])
    return nearest(_state["exact"]) if mm < EXACT_BELOW_MM else mm


def main():
    axes = {j: np.arange(lo, hi + step / 2, step) for j, (lo, hi, step) in AXES.items()}
    grid = [(e, f, r) for e in axes["elbow_flex"] for f in axes["wrist_flex"] for r in axes["wrist_roll"]]
    started = time.time()
    with multiprocessing.Pool(initializer=_init) as pool:
        mm = pool.map(clearance, grid, chunksize=64)
    print(f"{len(grid)} poses in {time.time() - started:.0f} s; {sum(v <= 0 for v in mm)} touching", file=sys.stderr)
    OUT.write_text(json.dumps({
        "about": "Wrist camera clearance to the follower's upper arm, forearm and wrist, in mm (0 = touching), "
                 "over the grid below. Row-major in the order of `joints`. Made by tools/camera_clearance.py.",
        "model": {"repo": REPO, "commit": COMMIT, "urdf": URDF, "mount": "Hex-Nut 32x32 UVC wrist mount"},
        "pairs": [list(p) for p in PAIRS],
        "joints": list(AXES),
        "axes": [{"min": lo, "step": step, "count": len(axes[j])} for j, (lo, hi, step) in AXES.items()],
        "clearance_mm": [round(v, 1) for v in mm],
    }, separators=(",", ":")) + "\n")
    print(f"wrote {OUT.relative_to(ROOT)}", file=sys.stderr)


if __name__ == "__main__":
    main()
