"""
Meshes the NeoRacer's CAD model (neoracer-model's parametric car, exported as
one STEP file with its colours) for AutoDRIVE: every solid tessellated, each
triangle with its face's colour, into one file that
Tools/neoracer_cad_fbx.py turns into the Unity model.

The STEP is neoracer-model's work/cad/neoracer-param.step (fc_param_export.py
and step_colours.py), with its colours beside it: neoracer-param-colours.json
(a colour per solid, by label) and neoracer-param-face-colours.json (a colour
per face, for the solids painted face by face: the LED matrix, a PCB). Its
frame: millimetres, y up, the car facing -z, the rear axle at z 0.

Run with FreeCAD's console, which opens any further argument as a file, so
the paths come in the environment:
    CAD_STEP=<car.step> CAD_COLOURS=<colours.json> CAD_FACES=<face-colours.json>
    CAD_OUT=<out.npz> [CAD_TOLERANCE=<mm>] FreeCADCmd.exe Tools/neoracer_cad_mesh.py
<out.npz> holds the triangles and a JSON index beside it (<out>.json): every
solid's label, its category (the STEP's top-level groups), its triangle range
and its colours. CAD_TOLERANCE is the tessellation's largest distance from the
true surface (default 0.05 mm).
"""

import json
import os
import time

import FreeCAD
import Import
import numpy as np

step, colours_file, faces_file, out = (os.environ[k] for k in ("CAD_STEP", "CAD_COLOURS", "CAD_FACES", "CAD_OUT"))
tolerance = float(os.environ.get("CAD_TOLERANCE", "0.05"))
colours = json.load(open(colours_file, encoding="utf-8"))
face_colours = json.load(open(faces_file, encoding="utf-8"))

t0 = time.time()
doc = FreeCAD.newDocument("car")
Import.insert(step, doc.Name)
print("IMPORTED", len(doc.Objects), "objects in", round(time.time() - t0, 1), "s")

palette, palette_index = [], {}


def colour_id(c):
    key = tuple(round(float(v), 4) for v in (list(c) + [1.0])[:4])
    if key not in palette_index:
        palette_index[key] = len(palette)
        palette.append(list(key))
    return palette_index[key]


verts, tris, tri_colour, solids = [], [], [], []
n_verts = n_tris = 0
for o in doc.Objects:
    if o.TypeId != "Part::Feature":
        continue
    label = o.Label
    groups = [p.Label for p in o.InList if p.TypeId == "App::LinkGroup"]
    painted = face_colours.get(label)
    shape = o.Shape
    if painted is not None and len(painted) != len(shape.Faces):
        raise SystemExit(f"{label}: {len(shape.Faces)} faces, {len(painted)} face colours")
    start_t, start_v = n_tris, n_verts
    for i, face in enumerate(shape.Faces):
        points, faces = face.tessellate(tolerance)
        if not faces:
            continue
        p = np.array([[v.x, v.y, v.z] for v in points], dtype=np.float32)
        f = np.array(faces, dtype=np.int32) + n_verts
        verts.append(p)
        tris.append(f)
        tri_colour.append(np.full(len(f), colour_id(painted[i] if painted else colours[label]), dtype=np.int16))
        n_verts += len(p)
        n_tris += len(f)
    solids.append({"label": label, "category": groups[0] if groups else "", "tris": [start_t, n_tris],
                   "verts": [start_v, n_verts]})

np.savez_compressed(out, verts=np.concatenate(verts), tris=np.concatenate(tris), colour=np.concatenate(tri_colour))
index = {"source": step, "tolerance_mm": tolerance, "frame": "mm, y up, the car facing -z, the rear axle at z 0",
         "palette": palette, "solids": solids}
json.dump(index, open(out.rsplit(".", 1)[0] + ".json", "w", encoding="utf-8"), indent=0)
print("MESHED", len(solids), "solids,", n_tris, "triangles,", len(palette), "colours at", tolerance, "mm in",
      round(time.time() - t0, 1), "s")
