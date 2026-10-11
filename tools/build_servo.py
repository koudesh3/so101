#!/usr/bin/env python3
"""Pack the two STS3215 shells at full resolution.

model.bin decimates the servos hard, and at that face count some of their
faces stop being flat: one triangle of a face tilts away from its neighbour
and catches the cel shader's hard highlight on its own, as a white wedge.
This writes both shells undecimated, with the same vertex layout and the same
crease-aware and hull normals as build_model.py, for the dashboard and the
viewer to draw in place of model.bin's copies:

  src/report/assets/servo.bin    each mesh's vertices then uint32 indices
  src/report/assets/servo.json   {"stride", "meshes": {name: {v_off, v_count, i_off, i_count}}}

usage: tools/build_servo.py path/to/SO-ARM100/Simulation/SO101
needs: numpy
"""
import json
import pathlib
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import build_model  # noqa: E402

NAMES = ["sts3215_03a_v1.stl", "sts3215_03a_no_horn_v1.stl"]


def main():
    src = pathlib.Path(sys.argv[1])
    out_dir = pathlib.Path(__file__).resolve().parent.parent / "src" / "report" / "assets"
    blob, meshes = bytearray(), {}
    for name in NAMES:
        points, faces = build_model.load_stl(src / "assets" / name)
        pos, nrm, hull, idx = build_model.build_vertices(points, faces)
        v_off = len(blob)
        for p, n, h in zip(pos, nrm, hull):
            blob += struct.pack("<3f3bx3bx", *p, *n, *h)
        i_off = len(blob)
        blob += idx.tobytes()
        meshes[name] = {"v_off": v_off, "v_count": len(pos), "i_off": i_off, "i_count": len(idx)}
        print(f"{name:32s} {len(idx) // 3:6d} faces {len(pos):6d} verts")
    (out_dir / "servo.bin").write_bytes(bytes(blob))
    (out_dir / "servo.json").write_text(json.dumps({"stride": 20, "meshes": meshes}, separators=(",", ":")))
    print(f"servo.bin {len(blob) / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
