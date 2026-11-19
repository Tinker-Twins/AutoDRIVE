"""
The NeoRacer's finished CAD model as AutoDRIVE's car model: the triangles
Tools/neoracer_cad_mesh.py took from the STEP, simplified to a budget a
60 fps simulator can draw, with materials, as one FBX for Unity and a JSON
that NeoRacerModelSwap reads.

blender -b --python Tools/neoracer_cad_fbx.py -- MESH_NPZ LED_FBX LED_JSON LED_TEXTURES OUT_FBX

MESH_NPZ      neoracer_cad_mesh.py's output (its JSON index beside it)
LED_FBX/JSON  an earlier model of this script's (Assets/Models/Vehicle/NeoRacer/CAD/
              NeoRacer CAD.fbx and .json) whose part "led", the LED matrix with
              its dot texture, replaces the CAD's plain one; LED_TEXTURES holds
              its texture (that folder's Textures). Its first build took the
              Playground race build's LED matrix.
              Copy LED_FBX/JSON/TEXTURES away before writing OUT_FBX over them.
OUT_FBX       the model; OUT_FBX's .json and a Textures folder beside it

The FBX, for Unity: x right, y up, z forward, metres, the axis change baked
into the meshes so every part imports unrotated, the origin at the middle of
the rear axle.
  Wheel FL, Wheel FR, Wheel RL, Wheel RR  each corner's tyre, rim, nut, stud and
                                          drive hex, origin at the tyre's centre
  led                                     the earlier model's LED matrix
  sidepods                                the sidepods with their marks and NEORACER words
  every other part                        one per top-level group of the STEP
The JSON gives the wheels' centres, radii and widths, the camera's and the
LiDAR's places (the bounding-box centres of the camera's lens and of the
LiDAR, as the Playground's measure-car.mjs took its sensors' from the race
build), the camera's lean (its glass's facing), the body's collider (the convex hull of everything but the
wheels, simplified under Unity's 255 triangles for a convex mesh collider,
and its fit), and every
material: the CAD's colour as linear base colour, with a finish (metal,
rubber, glass and so on) by what the part is. Places are in Blender's frame:
x right, y forward, z up.
"""
import json
import math
import os
import re
import shutil
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

mesh_npz, led_fbx, led_json, led_textures, out_fbx = sys.argv[sys.argv.index('--') + 1:][:5]
bpy.ops.wm.read_factory_settings(use_empty=True)
index = json.load(open(mesh_npz.rsplit('.', 1)[0] + '.json', encoding='utf-8'))
data = np.load(mesh_npz)
# STEP: millimetres, y up, the car facing -z, x its right. Blender: metres, x right, y forward, z up.
VERTS = np.stack([data['verts'][:, 0], -data['verts'][:, 2], data['verts'][:, 1]], axis=1).astype(np.float64) / 1000.0
TRIS = data['tris']
COLOUR = data['colour']
PALETTE = index['palette']

GROUPS = {'chassis': 'chassis', 'electronics': 'electronics', 'printed': 'printed', 'fasteners': 'fasteners',
          'wiring': 'wiring', 'look layer (vendor models)': 'vendor', 'look layer (decals)': 'decals'}
# Triangles each part keeps: about 710,000 in all with the LED matrix, against 244,844 for the
# Playground's race build. Each wheel keeps its tread; the screws, smallest on screen, the fewest.
BUDGET = {'Wheel': 40000, 'chassis': 200000, 'vendor': 100000, 'wiring': 50000, 'printed': 50000,
          'fasteners': 25000, 'electronics': 30000, 'decals': None, 'sidepods': 80000}
# The sidepods with their marks and NEORACER words, a part of their own. Mark and word are inlays,
# flush with the sidepod's face in letter-shaped holes; cut down with the other printed parts (to
# about a ninth of their triangles) the word did not show and the mark only in pieces.
SIDEPODS = r'^sidepod \d+( mark| word)?$'
DROPPED = {'led_matrix 1'}  # the LED matrix comes from LED_FBX
# On each wheel's axis, outboard of the knuckle (all of it over 90 mm from the middle; the
# differentials have drive hex pins too): they spin and steer with the wheel
ON_THE_WHEEL = r'^(drive_hex|drive_hex_pin|wheel_stud) \d+$'
KNUCKLE_OUTBOARD = 0.09
HULL_TRIANGLES = 250


def srgb_to_linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


# A part's finish: (pattern on its label, or None, and its CAD colour, or None) -> finish. The
# first that matches. Values follow the Playground race build's materials for the same things.
FINISHES = [
    (r'^camera_glass', None, dict(kind='glass', metallic=0.0, roughness=0.05)),
    (r'^plug_rj45 ', None, dict(kind='clear', metallic=0.0, roughness=0.08)),
    (r'^tyre_', None, dict(kind='rubber', metallic=0.0, roughness=0.92)),
    (r'^bumper_foam', None, dict(kind='foam', metallic=0.0, roughness=0.9)),
    (r'^shock_o_ring', None, dict(kind='rubber', metallic=0.0, roughness=0.8)),
    (r'^BATT_SEG#on ', None, dict(kind='glow', metallic=0.0, roughness=0.4, emission=[0.044, 1.0, 0.195], strength=3.0)),
    (r'^jetson_led', None, dict(kind='glow', metallic=0.0, roughness=0.2, emission=[0.065, 1.0, 0.042], strength=4.0)),
    (r'^cup_washer', None, dict(kind='anodised', metallic=0.85, roughness=0.3)),
    (r'heat_sink', None, dict(kind='anodised', metallic=0.85, roughness=0.3)),
    (r'^port_usba_tongue', None, dict(kind='plastic', metallic=0.0, roughness=0.5)),
    (None, (0.769, 0.773, 0.78), dict(kind='steel', metallic=1.0, roughness=0.4)),
    (None, (0.722, 0.729, 0.741), dict(kind='steel', metallic=1.0, roughness=0.4)),
    (None, (0.706, 0.706, 0.682), dict(kind='steel', metallic=1.0, roughness=0.4)),
    (None, (0.788, 0.776, 0.737), dict(kind='steel', metallic=1.0, roughness=0.4)),
    (None, (0.784, 0.784, 0.769), dict(kind='steel', metallic=1.0, roughness=0.4)),
    (None, (0.165, 0.169, 0.18), dict(kind='blackoxide', metallic=0.6, roughness=0.45)),
    (None, (0.761, 0.173, 0.094), dict(kind='anodised', metallic=0.85, roughness=0.3)),
    (None, (0.788, 0.635, 0.369), dict(kind='anodised', metallic=0.9, roughness=0.32)),
    (None, (0.831, 0.686, 0.322), dict(kind='gold', metallic=1.0, roughness=0.3)),
    (None, (0.604, 0.478, 0.369), dict(kind='brass', metallic=1.0, roughness=0.35)),
]
PLASTIC = dict(kind='plastic', metallic=0.0, roughness=0.55)


def finish(label, category, rgba):
    rgb = tuple(round(v, 3) for v in rgba[:3])
    for pattern, colour, f in FINISHES:
        if pattern is not None and re.search(pattern, label):
            return f
        if colour is not None and pattern is None and rgb == colour:
            return f
    return PLASTIC


materials = {}  # name -> spec for the JSON
material_order = []


def material_for(label, category, colour_id):
    rgba = PALETTE[colour_id]
    f = finish(label, category, rgba)
    decal = category == 'look layer (decals)'
    name = '%s_%02x%02x%02x%s' % (f['kind'], *(round(v * 255) for v in rgba[:3]), '_2s' if decal else '')
    if name not in materials:
        materials[name] = {
            'name': name,
            'baseColor': [round(srgb_to_linear(v), 5) for v in rgba[:3]] + [1.0],
            'metallic': f['metallic'], 'roughness': f['roughness'],
            'emission': f.get('emission', [1.0, 1.0, 1.0]), 'emissionStrength': f.get('strength', 0.0),
            'alpha': round(rgba[3], 4), 'doubleSided': decal, 'texture': '',
        }
        bpy.data.materials.new(name)
        material_order.append(name)
    return name


def corner_of(solid):
    lo = VERTS[slice(*solid['verts'])].min(0)
    hi = VERTS[slice(*solid['verts'])].max(0)
    centre = (lo + hi) / 2
    return ('F' if centre[1] > 0.144 else 'R') + ('L' if centre[0] < 0 else 'R')


def bbox(solids):
    pts = np.concatenate([VERTS[slice(*s['verts'])] for s in solids])
    return pts.min(0), pts.max(0)


# Every solid to its part
parts = {}
for s in index['solids']:
    if s['label'] in DROPPED:
        continue
    on_the_wheel = re.match(ON_THE_WHEEL, s['label']) and np.abs(VERTS[slice(*s['verts']), 0]).min() > KNUCKLE_OUTBOARD
    if s['category'] == 'wheel' or on_the_wheel:
        part = 'Wheel ' + corner_of(s)
    elif re.match(SIDEPODS, s['label']):
        part = 'sidepods'
    else:
        part = GROUPS[s['category']]
    parts.setdefault(part, []).append(s)

wheels = {}
for name in ('Wheel FL', 'Wheel FR', 'Wheel RL', 'Wheel RR'):
    tyre = [s for s in parts[name] if s['label'].startswith('tyre_')]
    lo, hi = bbox(tyre)
    wheels[name] = {'centre': (lo + hi) / 2, 'radius': (hi[2] - lo[2]) / 2, 'width': hi[0] - lo[0]}
origin = (wheels['Wheel RL']['centre'] + wheels['Wheel RR']['centre']) / 2  # the middle of the rear axle
VERTS -= origin


SOLID_OF_VERT = np.empty(len(VERTS), dtype=np.int64)
for i, s in enumerate(index['solids']):
    SOLID_OF_VERT[slice(*s['verts'])] = i


def weld(verts, solid, tris):
    """Merges each solid's vertices within 0.01 mm (the CAD meshes each face apart), never two
    solids' (where they touch, the edges would join more than two faces), and drops the triangles
    that collapse."""
    key = np.concatenate([solid[:, None], np.round(verts / 1e-5).astype(np.int64)], axis=1)
    _, first, inverse = np.unique(key, axis=0, return_index=True, return_inverse=True)
    tris = inverse.reshape(-1)[tris]
    keep = (tris[:, 0] != tris[:, 1]) & (tris[:, 1] != tris[:, 2]) & (tris[:, 0] != tris[:, 2])
    return verts[first], tris, keep


def edge_counts(tris):
    e = np.sort(np.concatenate([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]]), axis=1)
    _, n = np.unique(e, axis=0, return_counts=True)
    return int((n == 1).sum()), int((n > 2).sum())


def decimate(ob, ratio):
    mod = ob.modifiers.new('decimate', 'DECIMATE')
    mod.decimate_type = 'COLLAPSE'
    mod.ratio = ratio
    mod.use_collapse_triangulate = True
    with bpy.context.temp_override(object=ob, active_object=ob):
        bpy.ops.object.modifier_apply(modifier=mod.name)


def build(name, solids, budget, pivot=None):
    tri_list, mat_list, slots = [], [], []
    for s in solids:
        tri_list.append(TRIS[slice(*s['tris'])])
        cols = COLOUR[slice(*s['tris'])]
        lut = np.full(len(PALETTE), -1, dtype=np.int32)
        for c in np.unique(cols):
            m = material_for(s['label'], s['category'], int(c))
            if m not in slots:
                slots.append(m)
            lut[c] = slots.index(m)
        mat_list.append(lut[cols])
    tris = np.concatenate(tri_list)
    mats = np.concatenate(mat_list)
    used = np.unique(tris)
    remap = np.full(len(VERTS), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    verts, tris, keep = weld(VERTS[used], SOLID_OF_VERT[used], remap[tris])
    tris, mats = tris[keep], mats[keep]
    open_edges, shared_edges = edge_counts(tris)
    if pivot is not None:
        verts = verts - pivot

    me = bpy.data.meshes.new(name)
    me.vertices.add(len(verts))
    me.vertices.foreach_set('co', verts.astype(np.float32).ravel())
    me.loops.add(len(tris) * 3)
    me.loops.foreach_set('vertex_index', tris.astype(np.int32).ravel())
    me.polygons.add(len(tris))
    me.polygons.foreach_set('loop_start', np.arange(0, len(tris) * 3, 3, dtype=np.int32))
    me.polygons.foreach_set('material_index', mats.astype(np.int32))
    me.update(calc_edges=True)
    for m in slots:
        me.materials.append(bpy.data.materials[m])
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    if pivot is not None:
        ob.location = Vector(pivot)
    before = len(me.polygons)
    if budget is not None and before > budget:
        decimate(ob, budget / before)
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(30))
    print('PART %-12s %4d solids %9d -> %7d triangles, %d materials; welded: %d open edges, %d on 3+ faces' % (
        name, len(solids), before, len(me.polygons), len(slots), open_edges, shared_edges))
    return ob


for name in ('Wheel FL', 'Wheel FR', 'Wheel RL', 'Wheel RR'):
    build(name, parts[name], BUDGET['Wheel'], pivot=wheels[name]['centre'] - origin)
for name in list(GROUPS.values()) + ['sidepods']:
    build(name, parts[name], BUDGET[name])

# The LED matrix: the earlier model's, where the CAD has its own (the same part, 32 x 34 x 31.7 mm)
cad_led = next(s for s in index['solids'] if s['label'] == 'led_matrix 1')
cad_lo, cad_hi = VERTS[slice(*cad_led['verts'])].min(0), VERTS[slice(*cad_led['verts'])].max(0)
ours = set(bpy.data.objects)
bpy.ops.import_scene.fbx(filepath=led_fbx)
imported = [o for o in bpy.data.objects if o not in ours]
led = next(o for o in imported if o.name == 'led')
# The model FBXs leave turned half round (see the export below): turn it back
led.data.transform(Matrix.Rotation(math.pi, 4, 'Z') @ led.matrix_world)
led.parent = None
led.matrix_world = Matrix.Identity(4)
for o in imported:
    if o is not led:
        bpy.data.objects.remove(o)
pts = np.array([v.co[:] for v in led.data.vertices])
lo, hi = pts.min(0), pts.max(0)
size_gap = np.abs((hi - lo) - (cad_hi - cad_lo)).max()
if size_gap > 0.0005:
    raise SystemExit('the LED matrix of %s is %s, the CAD\'s %s' % (led_fbx, hi - lo, cad_hi - cad_lo))
led.data.transform(Matrix.Translation(Vector((cad_lo + cad_hi) / 2 - (lo + hi) / 2)))


def mean_normal(normals, areas):
    n = (normals * areas[:, None]).sum(0)
    return n / np.linalg.norm(n)


# Its lit face must face the way the CAD's does (the CAD paints the dots white)
dot_slot = [i for i, s in enumerate(led.material_slots) if s.material.name == 'led_dot'][0]
polys = [p for p in led.data.polygons if p.material_index == dot_slot]
ours_n = mean_normal(np.array([p.normal[:] for p in polys]), np.array([p.area for p in polys]))
t = TRIS[slice(*cad_led['tris'])]
white = [i for i, c in enumerate(PALETTE) if tuple(round(v, 3) for v in c[:3]) == (0.949, 0.949, 0.949)]
t = t[np.isin(COLOUR[slice(*cad_led['tris'])], white)]
a, b, c = VERTS[t[:, 0]], VERTS[t[:, 1]], VERTS[t[:, 2]]
cross = np.cross(b - a, c - a)
cad_n = mean_normal(cross / np.linalg.norm(cross, axis=1)[:, None], np.linalg.norm(cross, axis=1) / 2)
if float(ours_n @ cad_n) < 0.95:
    raise SystemExit('the LED matrix faces %s, the CAD\'s dots %s' % (ours_n, cad_n))
led_spec = {m['name']: m for m in json.load(open(led_json, encoding='utf-8'))['materials']}
textures = os.path.join(os.path.dirname(out_fbx), 'Textures')
os.makedirs(textures, exist_ok=True)
for slot in led.material_slots:
    m = dict(led_spec[slot.material.name])
    materials[m['name']] = m
    material_order.append(m['name'])
    if m['texture']:
        shutil.copyfile(os.path.join(led_textures, m['texture']), os.path.join(textures, m['texture']))
print('PART led          from %s, %d triangles, moved %s m, faces the CAD\'s dots within %.1f degrees' % (
    led_fbx, len(led.data.polygons), np.round((cad_lo + cad_hi) / 2 - (lo + hi) / 2, 5),
    math.degrees(math.acos(min(1.0, float(ours_n @ cad_n))))))

# The body's collider: the convex hull of everything but the wheels, simplified, its fit measured
body = np.concatenate([np.array([o.matrix_world @ v.co for v in o.data.vertices]) for o in bpy.data.objects
                       if o.type == 'MESH' and not o.name.startswith('Wheel')])


def hull_of(points, me):
    """Writes the convex hull of points to me, as triangles; its volume."""
    bm = bmesh.new()
    for p in points:
        bm.verts.new(p)
    hull = bmesh.ops.convex_hull(bm, input=bm.verts)
    inside = {g for g in hull['geom_interior'] + hull['geom_unused'] if isinstance(g, bmesh.types.BMVert)}
    bmesh.ops.delete(bm, geom=list(inside), context='VERTS')
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    volume = bm.calc_volume()
    bm.to_mesh(me)
    bm.free()
    return volume


me = bpy.data.meshes.new('Collider Body')
full_volume = hull_of(body, me)
full = len(me.polygons)
hull_ob = bpy.data.objects.new('Collider Body', me)
bpy.context.scene.collection.objects.link(hull_ob)
volume = full_volume
while len(me.polygons) > HULL_TRIANGLES:
    decimate(hull_ob, 0.9 * HULL_TRIANGLES / len(me.polygons))
    volume = hull_of([v.co.copy() for v in me.vertices], me)
# How far the body reaches out of the simplified hull: over each face's plane, outward
hv = np.array([v.co[:] for v in me.vertices])
tri = np.array([p.vertices[:] for p in me.polygons])
n = np.cross(hv[tri[:, 1]] - hv[tri[:, 0]], hv[tri[:, 2]] - hv[tri[:, 0]])
n /= np.linalg.norm(n, axis=1)[:, None]
n *= np.sign(((hv[tri].mean(1) - hv.mean(0)) * n).sum(1))[:, None]
d = -(n * hv[tri[:, 0]]).sum(1)
outside = (body @ n.T + d).max(1)
collider = {'vertices': [round(float(v), 5) for v in hv.ravel()], 'triangles': [int(i) for i in tri.ravel()],
            'full_hull_triangles': full, 'body_outside_max': round(float(outside.max()), 5),
            'volume_of_full_hull': round(volume / full_volume, 4)}
print('PART collider     %d triangles (the full hull %d), %.1f%% of its volume; the body reaches %.1f mm outside it' % (
    len(tri), full, 100 * volume / full_volume, 1000 * collider['body_outside_max']))
bpy.data.objects.remove(hull_ob)


def centre_of(prefixes):
    lo, hi = bbox([s for s in index['solids'] if s['label'].startswith(prefixes)])
    return [round(float(v), 5) for v in (lo + hi) / 2]


camera = centre_of(('camera_lens',))  # where the image forms, not the module's middle (16 mm behind it)
lidar = centre_of(('RC-LIDAR', 'lidar_led'))
# The lens's lean: the camera glass's front face, how far it looks up from level
glass = next(s for s in index['solids'] if s['label'] == 'camera_glass 1')
t = TRIS[slice(*glass['tris'])]
cross = np.cross(VERTS[t[:, 1]] - VERTS[t[:, 0]], VERTS[t[:, 2]] - VERTS[t[:, 0]])
front = cross[cross[:, 1] > 0.5 * np.linalg.norm(cross, axis=1)]
axis = front.sum(0) / np.linalg.norm(front.sum(0))
camera_lean = round(math.degrees(math.asin(axis[2])), 2)

# Unity's importer lands Blender's -Y on Unity's +Z (with forward '-Z'): turn the car half round
# the vertical so its front (+Y here) leaves as -Y, and every part imports unrotated with the
# front on Unity's +Z and the right on +X. Forward 'Z' writes an FBX whose axes are turned half
# round, and Unity turns every part 180 degrees to undo it: AutoDRIVE, which sets each wheel's
# rotation from its collider, then shows the wheels the wrong way round.
turn = Matrix.Rotation(math.pi, 4, 'Z')
for ob in bpy.data.objects:
    if ob.type == 'MESH':
        ob.matrix_world = turn @ ob.matrix_world
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
# Blender writes every texture's absolute path, in the FBX's folder on this machine, beside its
# relative one, whatever path_mode is: the FBX names only the file
from io_scene_fbx import export_fbx_bin
vid_paths = export_fbx_bin._gen_vid_path
export_fbx_bin._gen_vid_path = lambda img, scene_data: (vid_paths(img, scene_data)[1],) * 2
bpy.ops.export_scene.fbx(
    filepath=out_fbx, use_selection=True, object_types={'MESH'},
    apply_unit_scale=True, apply_scale_options='FBX_SCALE_UNITS', global_scale=1.0,
    axis_forward='-Z', axis_up='Y', bake_space_transform=True,
    mesh_smooth_type='FACE', use_mesh_modifiers=True, use_tspace=False,
    path_mode='STRIP', embed_textures=False, add_leaf_bones=False, bake_anim=False)

objects = sorted(o.name for o in bpy.data.objects if o.type == 'MESH')
spec = {
    'source': os.path.basename(index['source']) + ', meshed at %s mm' % index['tolerance_mm'],
    'triangles': sum(len(o.data.polygons) for o in bpy.data.objects if o.type == 'MESH'),
    'parts': objects,
    'collider': collider,
    'wheels': [{'name': n, 'centre': [round(float(v), 5) for v in w['centre'] - origin],
                'radius': round(float(w['radius']), 5), 'width': round(float(w['width']), 5)} for n, w in wheels.items()],
    'camera': camera,
    'camera_lean_degrees': camera_lean,
    'lidar': lidar,
    'materials': [materials[n] for n in material_order],
}
json.dump(spec, open(out_fbx.rsplit('.', 1)[0] + '.json', 'w', encoding='utf-8'), indent=1)
print('EXPORTED', out_fbx, len(objects), 'parts', spec['triangles'], 'triangles drawn', len(material_order), 'materials')
print('WHEELS', json.dumps(spec['wheels']))
print('CAMERA', spec['camera'], 'leaning up', camera_lean, 'degrees; LIDAR', spec['lidar'])
