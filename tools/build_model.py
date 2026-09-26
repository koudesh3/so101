#!/usr/bin/env python3
"""Pack the SO-101 meshes for the dashboard.

Reads TheRobotStudio's URDFs and STLs (Apache-2.0,
https://github.com/TheRobotStudio/SO-ARM100, Simulation/SO101) and writes:

  src/report/assets/model.bin   every mesh, decimated, as interleaved vertices + indices
  src/report/assets/model.json  the kinematic tree of both arms and where each mesh sits

Vertex layout, 20 bytes: position f32x3 | shading normal i8x3 + pad | hull normal i8x3 + pad.
The shading normal is crease-aware (faces meeting at more than CREASE_DEG keep
their own normal), so cel bands break cleanly at CAD edges. The hull normal is
fully smoothed, so the outline pass can push the surface out without cracks.

usage: tools/build_model.py path/to/SO-ARM100/Simulation/SO101
needs: numpy, fast_simplification
"""
import json
import math
import pathlib
import struct
import sys
import xml.etree.ElementTree as ET

import fast_simplification
import numpy as np

CREASE_DEG = 38.0
MAX_FACES = {"sts3215_03a_v1.stl": 3200, "sts3215_03a_no_horn_v1.stl": 3000}
DEFAULT_MAX_FACES = 6000

# Which joint each link's servo drives (the motor that lives in that link).
SERVO_JOINT = {"base_link": 0, "shoulder_link": 1, "upper_arm_link": 2, "lower_arm_link": 3, "wrist_link": 4, "gripper_link": 5}


def load_stl(path):
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84 : 84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    tri = rec["v"].astype(np.float64)
    flat = tri.reshape(-1, 3)
    keys = np.round(flat / 1e-6).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    points = flat[first]
    faces = inverse.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    return points, faces


def decimate(points, faces, max_faces):
    if len(faces) <= max_faces:
        return points, faces
    reduction = 1.0 - max_faces / len(faces)
    p, f = fast_simplification.simplify(points.astype(np.float32), faces.astype(np.int32), target_reduction=reduction)
    return p.astype(np.float64), f.astype(np.int64)


def build_vertices(points, faces):
    a, b, c = (points[faces[:, i]] for i in range(3))
    cross = np.cross(b - a, c - a)
    area = np.linalg.norm(cross, axis=1)
    keep = area > 1e-14
    faces, cross, area = faces[keep], cross[keep], area[keep]
    fn = cross / area[:, None]

    # Faces around each vertex.
    around = [[] for _ in range(len(points))]
    for fi, tri in enumerate(faces):
        for v in tri:
            around[v].append(fi)

    hull = np.zeros_like(points)
    for v, fs in enumerate(around):
        if fs:
            s = (fn[fs] * area[fs, None]).sum(0)
            hull[v] = s / (np.linalg.norm(s) or 1.0)

    cos_crease = math.cos(math.radians(CREASE_DEG))
    out_pos, out_n, out_h, index = [], [], [], []
    lookup = {}
    for fi, tri in enumerate(faces):
        for v in tri:
            fs = around[v]
            near = [g for g in fs if fn[g] @ fn[fi] >= cos_crease]
            s = (fn[near] * area[near, None]).sum(0)
            n = s / (np.linalg.norm(s) or 1.0)
            q = tuple(np.round(n * 127).astype(int))
            key = (v, q)
            if key not in lookup:
                lookup[key] = len(out_pos)
                out_pos.append(points[v])
                out_n.append(q)
                out_h.append(tuple(np.round(hull[v] * 127).astype(int)))
            index.append(lookup[key])
    return np.array(out_pos, np.float32), np.array(out_n, np.int8), np.array(out_h, np.int8), np.array(index, np.uint32)


def floats(s):
    return [float(x) for x in s.split()] if s else [0.0, 0.0, 0.0]


def parse_urdf(path):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        visuals = []
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            origin = vis.find("origin")
            mat = vis.find("material")
            name = mesh.get("filename").split("/")[-1]
            visuals.append({
                "mesh": name,
                "xyz": floats(origin.get("xyz") if origin is not None else ""),
                "rpy": floats(origin.get("rpy") if origin is not None else ""),
                "kind": "servo" if (mat is not None and mat.get("name") == "sts3215") else "printed",
                "joint": SERVO_JOINT.get(link.get("name"), -1) if (mat is not None and mat.get("name") == "sts3215") else -1,
            })
        links[link.get("name")] = visuals
    joints = []
    for j in root.findall("joint"):
        origin = j.find("origin")
        limit = j.find("limit")
        joints.append({
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": floats(origin.get("xyz")),
            "rpy": floats(origin.get("rpy")),
            "lower": float(limit.get("lower")) if limit is not None else 0.0,
            "upper": float(limit.get("upper")) if limit is not None else 0.0,
        })
    return {"links": links, "joints": joints}


def main():
    src = pathlib.Path(sys.argv[1])
    out_dir = pathlib.Path(__file__).resolve().parent.parent / "src" / "report" / "assets"
    out_dir.mkdir(parents=True, exist_ok=True)
    arms = {"follower": parse_urdf(src / "so101_new_calib.urdf"), "leader": parse_urdf(src / "so101_leader_new_calib.urdf")}
    names = sorted({v["mesh"] for arm in arms.values() for vs in arm["links"].values() for v in vs})

    blob = bytearray()
    meshes = {}
    for name in names:
        points, faces = load_stl(src / "assets" / name)
        points, faces = decimate(points, faces, MAX_FACES.get(name, DEFAULT_MAX_FACES))
        pos, nrm, hull, idx = build_vertices(points, faces)
        while len(blob) % 4:
            blob.append(0)
        v_off = len(blob)
        for p, n, h in zip(pos, nrm, hull):
            blob += struct.pack("<3f3bx3bx", *p, *n, *h)
        i_off = len(blob)
        blob += idx.tobytes()
        meshes[name] = {"v_off": v_off, "v_count": len(pos), "i_off": i_off, "i_count": len(idx)}
        print(f"{name:40s} {len(idx) // 3:6d} faces {len(pos):6d} verts")

    (out_dir / "model.bin").write_bytes(bytes(blob))
    (out_dir / "model.json").write_text(json.dumps({"stride": 20, "meshes": meshes, "arms": arms}, separators=(",", ":")))
    print(f"model.bin {len(blob) / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
