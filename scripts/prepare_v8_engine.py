"""Prepare the V8 engine mesh of the force module examples (headless Blender).

usage: blender -b --factory-startup -P scripts/prepare_v8_engine.py -- \
         scene.gltf examples/force_modules/meshes

Reads the glTF of "V8 Engine" by Tahminayeasminmony (Sketchfab, CC-BY-4.0,
see examples/force_modules/meshes/v8_engine_license.txt), 47 parts and 2.2
million triangles, and writes v8_engine.obj and v8_engine.mtl:

* each part is decimated, to about 2% of its triangles but at least 300 (or
  all of them for a smaller part), which keeps the bolts, the wiring and the
  turbo housings recognizable at about 50 thousand triangles in all;
* the engine is turned to the axes of shift: x forward along the crankshaft,
  with the transmission at the rear, y to the right and z down, in meters,
  and its origin is the center of the block;
* the white material of the source is made a dark grey, so the engine
  contrasts with the floor.
"""

import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

argv = sys.argv[sys.argv.index("--") + 1:]
source, out_dir = argv[0], argv[1]

KEEP = 0.02       # fraction of the triangles of a part kept
MINIMUM = 300     # but at least this many (or the part's own count)

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=source)

meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]

# bake every part's placement into its vertices and drop the hierarchy
for o in meshes:
    world = o.matrix_world.copy()
    o.parent = None
    o.data.transform(world)
    o.matrix_world = Matrix.Identity(4)
for o in [o for o in bpy.context.scene.objects if o.type != "MESH"]:
    bpy.data.objects.remove(o)

# Blender (Z up): the crankshaft runs along y with the transmission at +y.
# shift: x = -y, y = -x, z = -z (a proper rotation)
to_shift = Matrix(((0, -1, 0, 0), (-1, 0, 0, 0), (0, 0, -1, 0), (0, 0, 0, 1)))
for o in meshes:
    o.data.transform(to_shift)


def triangles(o):
    return sum(len(p.vertices) - 2 for p in o.data.polygons)


def bounds(objs):
    lo = Vector((1e9, 1e9, 1e9))
    hi = Vector((-1e9, -1e9, -1e9))
    for o in objs:
        for v in o.data.vertices:
            for k in range(3):
                lo[k] = min(lo[k], v.co[k])
                hi[k] = max(hi[k], v.co[k])
    return lo, hi


# the origin: the center of the block (the part named Block in the source)
block = [o for o in meshes if o.name.startswith(("Object_13", "Block"))]
if not block:
    raise SystemExit("the block part was not found; names: " +
                     ", ".join(o.name for o in meshes))
lo, hi = bounds(block)
center = (lo + hi) / 2
for o in meshes:
    o.data.transform(Matrix.Translation(-center))

before = sum(triangles(o) for o in meshes)
for o in meshes:
    count = triangles(o)
    target = max(min(count, MINIMUM), KEEP * count)
    if target < count:
        bpy.context.view_layer.objects.active = o
        modifier = o.modifiers.new("decimate", "DECIMATE")
        modifier.ratio = target / count
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    bpy.ops.object.select_all(action="DESELECT")
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
after = sum(triangles(o) for o in meshes)
print(f"triangles {before} -> {after}")

# materials: the source's three, with the white one made a dark grey
for material in bpy.data.materials:
    bsdf = material.node_tree.nodes["Principled BSDF"]
    if material.name == "Material.001":      # unset in the source: white
        bsdf.inputs["Base Color"].default_value = (0.06, 0.06, 0.065, 1.0)
        bsdf.inputs["Metallic"].default_value = 0.0
        bsdf.inputs["Roughness"].default_value = 0.45

os.makedirs(out_dir, exist_ok=True)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.wm.obj_export(
    filepath=os.path.join(out_dir, "v8_engine.obj"),
    export_selected_objects=True,
    forward_axis="Y",   # the coordinates as they are
    up_axis="Z",
    export_uv=False,
    export_materials=True,
    export_pbr_extensions=True,
    export_object_groups=True,
)
lo, hi = bounds(meshes)
print("size (m):", [round(h - l, 3) for l, h in zip(lo, hi)])
print("from", [round(x, 3) for x in lo], "to", [round(x, 3) for x in hi])
