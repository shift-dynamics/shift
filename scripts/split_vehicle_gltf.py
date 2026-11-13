"""Splits a glTF vehicle into one glTF file per body of a multibody model.

Run with Blender (4.2 or later):

    blender --background --factory-startup --python split_vehicle_gltf.py -- \
        config.json source.gltf output_directory

Every mesh of the source is split into its loose pieces. Each piece goes to
the body of the first rule that selects it, or to the default body. A rule
selects a piece when the node or mesh name matches "mesh" (a regular
expression; every piece if omitted) and the piece lies entirely inside "box"
([x_min, x_max, y_min, y_max, z_min, z_max] in vehicle coordinates). Rules
whose body name contains a "mirror" key ("right_") also select, for the body
named with its value ("left_"), the pieces inside the box mirrored in y.

Vehicle coordinates are SAE: x forward, y right, z down, in meters, with the
origin on the ground between the axles. "scale" multiplies the coordinates of
the source (after Blender's glTF import, z up), and "axes" are the rows of
the rotation from those coordinates to vehicle coordinates.

Each body is written as <output_directory>/<body>.glb, with its textures
embedded (at most "max_texture_size" pixels across, in "image_format":
"AUTO" keeps PNG, "JPEG" is smaller), in the coordinates of
the body: vehicle coordinates less the body's origin given in "bodies". A
listing of every piece, with its body, goes to pieces.txt.

"cuts" (optional) cut the meshes whose node or mesh name matches "mesh" along
the plane through "point" normal to "normal" (vehicle coordinates) before
they are split into pieces, e.g. a twist beam axle into the halves that go to
the trailing arms of either side.

"translations" (optional) translate the meshes whose node or mesh name
matches "mesh" by "translation" (vehicle coordinates) before they are cut and
split, e.g. wheels modelled lower than the hubs of the suspension.

"images" (optional) replace images of the source, by name (as Blender names
them on import: usually the file name, e.g. "plate_baseColor.png"), with
image files, by
paths relative to the configuration file, e.g. a license plate drawn by the
example's generator. The replacement is mapped as the image it replaces, so it
must be laid out the same way.

{
  "scale": 1.0,
  "axes": [[0, 1, 0], [1, 0, 0], [0, 0, -1]],
  "max_texture_size": 1024,
  "image_format": "JPEG",
  "default": "chassis",
  "mirror": ["right_", "left_"],
  "bodies": {"chassis": [0, 0, 0], "right_front_wheel": [1.45, 0.88, -0.37]},
  "rules": [{"body": "right_front_wheel", "mesh": "Tire|Rim",
             "box": [1.0, 1.9, 0.6, 1.2, -0.8, 0.1]}]
}
"""

import json
import os
import re
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector


def mirrored(rule, mirror):
    """the rule for the other side, or None"""
    if not mirror or mirror[0] not in rule["body"]:
        return None
    other = dict(rule)
    other["body"] = rule["body"].replace(mirror[0], mirror[1])
    x0, x1, y0, y1, z0, z1 = rule["box"]
    other["box"] = [x0, x1, -y1, -y0, z0, z1]
    return other


def node_names(obj):
    names = []
    while obj:
        names.append(obj.name)
        obj = obj.parent
    return " ".join(names)


def cut(obj, point, normal):
    """cuts the mesh of obj along a plane, so its sides become loose pieces"""
    mesh = bmesh.new()
    mesh.from_mesh(obj.data)
    geom = mesh.verts[:] + mesh.edges[:] + mesh.faces[:]
    result = bmesh.ops.bisect_plane(mesh, geom=geom, dist=1e-6,
                                    plane_co=Vector(point),
                                    plane_no=Vector(normal))
    edges = [e for e in result["geom_cut"] if isinstance(e, bmesh.types.BMEdge)]
    bmesh.ops.split_edges(mesh, edges=edges)
    mesh.to_mesh(obj.data)
    mesh.free()


def split_loose(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.separate(type="LOOSE")
    bpy.ops.object.mode_set(mode="OBJECT")
    return list(bpy.context.selected_objects)


def main():
    argv = sys.argv[sys.argv.index("--") + 1:]
    config_path, source, output = argv[0], argv[1], argv[2]
    with open(config_path) as f:
        config = json.load(f)
    rules = []
    for rule in config["rules"]:
        rules.append(rule)
        other = mirrored(rule, config.get("mirror"))
        if other:
            rules.append(other)
    for rule in rules:
        if rule["body"] not in config["bodies"]:
            raise SystemExit("rule for unknown body " + rule["body"])
    default = config.get("default", "chassis")

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=source)
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    # pieces are edited in world coordinates, one object each
    bpy.ops.object.select_all(action="DESELECT")
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    bpy.ops.object.make_single_user(object=True, obdata=True)
    axes = Matrix(config["axes"]).to_4x4()
    to_vehicle = axes @ Matrix.Scale(config.get("scale", 1.0), 4)
    for o in meshes:
        o["source"] = node_names(o)
        o.data.transform(to_vehicle @ o.matrix_world)
    for o in meshes:
        o.parent = None
        o.matrix_world = Matrix.Identity(4)
    for o in [o for o in bpy.data.objects if o.type != "MESH"]:
        bpy.data.objects.remove(o)

    for t in config.get("translations", []):
        for o in meshes:
            if re.search(t["mesh"], o["source"]):
                o.data.transform(Matrix.Translation(Vector(t["translation"])))

    for c in config.get("cuts", []):
        for o in meshes:
            if re.search(c["mesh"], o["source"]):
                cut(o, c["point"], c["normal"])

    pieces = []
    for o in meshes:
        source_name = o["source"]
        parts = split_loose(o) if any(
            re.search(r.get("mesh", "."), source_name) for r in rules) else [o]
        for p in parts:
            p["source"] = source_name
            pieces.append(p)

    listing = []
    members = {name: [] for name in config["bodies"]}
    for p in pieces:
        points = [v.co for v in p.data.vertices]
        if not points:
            continue
        lo = Vector([min(q[i] for q in points) for i in range(3)])
        hi = Vector([max(q[i] for q in points) for i in range(3)])
        body = default
        for rule in rules:
            if "mesh" in rule and not re.search(rule["mesh"], p["source"]):
                continue
            b = rule["box"]
            if all(b[2 * i] <= lo[i] and hi[i] <= b[2 * i + 1]
                   for i in range(3)):
                body = rule["body"]
                break
        members[body].append(p)
        listing.append("%-28s %-40s c=(%7.3f %7.3f %7.3f) size=(%6.3f %6.3f "
                       "%6.3f) faces=%d" % ((body, p["source"][:40]) +
                       tuple((lo + hi) / 2) + tuple(hi - lo) +
                       (len(p.data.polygons),)))

    for name, path in config.get("images", {}).items():
        if name not in bpy.data.images:
            raise SystemExit("no image " + name + " in the source; it has " +
                             ", ".join(i.name for i in bpy.data.images))
        # the source's images are packed on import: load the replacement and
        # point every user of the source's image to it
        bpy.data.images[name].user_remap(bpy.data.images.load(os.path.join(
            os.path.dirname(os.path.abspath(config_path)), path)))

    largest = config.get("max_texture_size", 0)
    for image in bpy.data.images:
        w, h = image.size
        if largest and max(w, h) > largest:
            s = largest / max(w, h)
            image.scale(max(1, round(w * s)), max(1, round(h * s)))

    os.makedirs(output, exist_ok=True)
    for name, parts in members.items():
        if not parts:
            print("body %s: no pieces" % name)
            continue
        bpy.ops.object.select_all(action="DESELECT")
        for p in parts:
            p.select_set(True)
        bpy.context.view_layer.objects.active = parts[0]
        if len(parts) > 1:
            bpy.ops.object.join()
        joined = bpy.context.view_layer.objects.active
        joined.name = name
        joined.data.transform(
            Matrix.Translation(-Vector(config["bodies"][name])))
        bpy.ops.object.select_all(action="DESELECT")
        joined.select_set(True)
        bpy.ops.export_scene.gltf(
            filepath=os.path.join(output, name + ".glb"),
            export_format="GLB",
            export_image_format=config.get("image_format", "AUTO"),
            export_jpeg_quality=90,
            use_selection=True, export_yup=False, export_apply=True,
            export_animations=False, export_cameras=False,
            export_lights=False)
        print("body %s: %d faces" % (name, len(joined.data.polygons)))
        joined.hide_set(True)

    with open(os.path.join(output, "pieces.txt"), "w") as f:
        f.write("\n".join(sorted(listing)) + "\n")


main()
