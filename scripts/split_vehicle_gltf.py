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
"AUTO" keeps PNG, "JPEG" is smaller, of "jpeg_quality" from 0 to 100,
90 if not given), in the coordinates of
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

"projections" (optional) move the vertices of the meshes whose node or
mesh name matches "mesh" onto the nearest surface of the meshes matching
"onto", where that is within "distance" (m), e.g. the glow of a lamp painted
on cards that stand off its lens, laid on the lens under the lamp's glass.

"decals" (optional) paint the image of the meshes whose node or mesh name
matches "mesh" onto the images of the meshes matching "onto", where they lie
within "distance" (m) of each other: each texel of the faces of "onto" takes
the color of the nearest point of the faces of "mesh" that lie wholly within
"distance" of "onto", over its own where that is brighter than "cutoff" (the
largest of red, green and blue, from 0 to 1, blended over 0.2 about it), e.g.
the glow of a lamp painted on cards that stand off its lens, painted on the
lens, so that the cards can be left out. The images painted are those after
"images" replaces them.

"cutouts" (optional) make materials of the source, by name, cut-outs: the
alpha of the image of the material's base color becomes the brightness of
its color (the largest of red, green and blue), and the material is cut out
where that is below the cutoff (glTF's "alphaMode": "MASK"), e.g. the glow of
lamps painted on black, whose black would otherwise be drawn as black paint
around them. Their images are written as PNG, with the alpha.

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
import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
from mathutils.geometry import barycentric_transform


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


def base_image(material):
    """the image node of the base color of material"""
    shader = next(n for n in material.node_tree.nodes
                  if n.type == "BSDF_PRINCIPLED")
    color = shader.inputs["Base Color"]
    if not color.links or color.links[0].from_node.type != "TEX_IMAGE":
        raise SystemExit("material " + material.name +
                         " has no image as base color")
    return color.links[0].from_node


def image_array(image):
    """the pixels of image, rows from the bottom, RGBA from 0 to 1"""
    w, h = image.size
    a = np.empty(w * h * 4, np.float32)
    image.pixels.foreach_get(a)
    return a.reshape(h, w, 4)


def bilinear(pixels, u, v):
    """the RGB of image pixels (rows from the bottom) at texture coordinates
    u, v, interpolated bilinearly, wrapping round"""
    h, w = pixels.shape[:2]
    x, y = u * w - 0.5, v * h - 0.5
    x0, y0 = int(np.floor(x)), int(np.floor(y))
    fx, fy = x - x0, y - y0
    c = [[pixels[(y0 + j) % h, (x0 + i) % w, :3] for i in (0, 1)]
         for j in (0, 1)]
    return ((1 - fy) * ((1 - fx) * c[0][0] + fx * c[0][1]) +
            fy * ((1 - fx) * c[1][0] + fx * c[1][1]))


def project(meshes, rule):
    """moves the vertices of the meshes matching rule["mesh"] onto the
    nearest surface of the meshes matching rule["onto"] within
    rule["distance"]"""
    onto = [o for o in meshes if re.search(rule["onto"], o["source"])]
    if not onto:
        raise SystemExit("projections: no mesh matches " + rule["onto"])
    b = bmesh.new()
    for o in onto:
        b.from_mesh(o.data)
    surface = BVHTree.FromBMesh(b)
    b.free()
    for o in meshes:
        if not re.search(rule["mesh"], o["source"]):
            continue
        moved = 0
        for v in o.data.vertices:
            hit = surface.find_nearest(v.co, rule["distance"])[0]
            if hit is not None:
                v.co = hit
                moved += 1
        print("projections: %d of %d vertices of %s onto %s" %
              (moved, len(o.data.vertices), o["source"], rule["onto"]))


def decal(meshes, rule):
    """paints the image of the meshes matching rule["mesh"] onto the images
    of the meshes matching rule["onto"] (see "decals" above)"""
    distance = rule["distance"]
    onto = [o for o in meshes if re.search(rule["onto"], o["source"])]
    cards = [o for o in meshes if re.search(rule["mesh"], o["source"])]
    if not onto or not cards:
        raise SystemExit("decals: no mesh matches " + rule["onto"] + " or " +
                         rule["mesh"])

    def tree(objects):
        b = bmesh.new()
        for o in objects:
            b.from_mesh(o.data)
        t = BVHTree.FromBMesh(b)
        b.free()
        return t

    def near(t, point):
        return t.find_nearest(point, distance)[0] is not None

    # the faces of the cards wholly near the meshes painted on, triangles
    # with their texture coordinates and the card's image
    onto_tree = tree(onto)
    triangles, uvs, images = [], [], []
    for o in cards:
        picture = image_array(base_image(o.data.materials[0]).image)
        o.data.calc_loop_triangles()
        layer = o.data.uv_layers.active.data
        for t in o.data.loop_triangles:
            corners = [o.data.vertices[v].co for v in t.vertices]
            if all(near(onto_tree, c) for c in corners):
                triangles.append(corners)
                uvs.append([Vector(tuple(layer[k].uv) + (0.0,))
                            for k in t.loops])
                images.append(picture)
    b = bmesh.new()
    for corners in triangles:
        verts = [b.verts.new(c) for c in corners]
        b.faces.new(verts)
    b.verts.ensure_lookup_table()
    b.faces.ensure_lookup_table()
    card_tree = BVHTree.FromBMesh(b)
    b.free()

    painted = {}
    low, high = rule["cutoff"] - 0.1, rule["cutoff"] + 0.1
    for o in onto:
        o.data.calc_loop_triangles()
        layer = o.data.uv_layers.active.data
        for t in o.data.loop_triangles:
            corners = [o.data.vertices[v].co for v in t.vertices]
            if not all(near(card_tree, c) for c in corners):
                continue
            node = base_image(o.data.materials[t.material_index])
            if node.image.name not in painted:
                painted[node.image.name] = (node.image,
                                            image_array(node.image))
            pixels = painted[node.image.name][1]
            h, w = pixels.shape[:2]
            uv = np.array([tuple(layer[k].uv) for k in t.loops]) * (w, h)
            lo = np.floor(uv.min(axis=0) - 1).astype(int)
            hi = np.ceil(uv.max(axis=0) + 1).astype(int)
            xs, ys = np.meshgrid(np.arange(lo[0], hi[0]),
                                 np.arange(lo[1], hi[1]))
            p = np.stack([xs.ravel(), ys.ravel()], axis=-1) + 0.5
            m = np.array([uv[1] - uv[0], uv[2] - uv[0]]).T
            if abs(np.linalg.det(m)) < 1e-12:
                continue
            l12 = np.linalg.solve(m, (p - uv[0]).T).T
            bary = np.column_stack([1.0 - l12.sum(axis=1), l12])
            # the texels of the triangle, and half a texel beyond its edges
            slack = 0.5 / max(1.0, np.abs(np.linalg.det(m)) ** 0.5)
            for (x, y), weights in zip(p.astype(int),
                                       bary):
                if weights.min() < -slack or not (0 <= x < w and
                                                   0 <= y < h):
                    continue
                point = sum((float(c) * v for c, v in zip(weights, corners)),
                            Vector())
                hit, _, index, _ = card_tree.find_nearest(point, distance)
                if hit is None:
                    continue
                q = barycentric_transform(hit, *triangles[index],
                                          *uvs[index])
                picture = images[index]
                ph, pw = picture.shape[:2]
                color = bilinear(picture, q.x, q.y)
                a = min(1.0, max(0.0, (color.max() - low) / (high - low)))
                pixels[y, x, :3] += a * (color - pixels[y, x, :3])
    for name, (image, pixels) in painted.items():
        # a new image, packed: the source's, read from a file, would be
        # written from the file
        w, h = image.size
        new = bpy.data.images.new(name + "_decal", w, h)
        new.pixels.foreach_set(pixels.ravel())
        new.pack()
        image.user_remap(new)
        print("decals: %s painted on %s" % (rule["mesh"], name))


def cut_out(name, cutoff):
    """makes the material name a cut-out where the brightness of its base
    color is below cutoff: the alpha of its image is the brightness, fed to
    the alpha of the shader through alpha < cutoff and 1 - that, which the
    glTF exporter writes as "alphaMode": "MASK" with alphaCutoff cutoff"""
    material = bpy.data.materials.get(name)
    if material is None or not material.use_nodes:
        raise SystemExit("no material " + name + " with nodes in the source")
    nodes, links = material.node_tree.nodes, material.node_tree.links
    shader = next(n for n in nodes if n.type == "BSDF_PRINCIPLED")
    color = shader.inputs["Base Color"]
    if not color.links or color.links[0].from_node.type != "TEX_IMAGE":
        raise SystemExit("material " + name + " has no image as base color")
    texture = color.links[0].from_node
    source = texture.image
    pixels = list(source.pixels)
    for k in range(0, len(pixels), 4):
        pixels[k + 3] = max(pixels[k], pixels[k + 1], pixels[k + 2])
    # a new image with an alpha channel: the source's, read from a file
    # without one, would be written without it
    width, height = source.size
    image = bpy.data.images.new(source.name, width, height, alpha=True)
    image.pixels[:] = pixels
    image.alpha_mode = "STRAIGHT"
    image.pack()
    texture.image = image
    below = nodes.new("ShaderNodeMath")
    below.operation = "LESS_THAN"
    below.inputs[1].default_value = cutoff
    keep = nodes.new("ShaderNodeMath")
    keep.operation = "SUBTRACT"
    keep.inputs[0].default_value = 1.0
    links.new(texture.outputs["Alpha"], below.inputs[0])
    links.new(below.outputs[0], keep.inputs[1])
    links.new(keep.outputs[0], shader.inputs["Alpha"])


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

    for r in config.get("projections", []):
        project([o for o in bpy.data.objects if o.type == "MESH"], r)

    for d in config.get("decals", []):
        decal([o for o in bpy.data.objects if o.type == "MESH"], d)

    for name, cutoff in config.get("cutouts", {}).items():
        cut_out(name, cutoff)

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
            export_jpeg_quality=config.get("jpeg_quality", 90),
            use_selection=True, export_yup=False, export_apply=True,
            export_animations=False, export_cameras=False,
            export_lights=False)
        print("body %s: %d faces" % (name, len(joined.data.polygons)))
        joined.hide_set(True)

    with open(os.path.join(output, "pieces.txt"), "w") as f:
        f.write("\n".join(sorted(listing)) + "\n")


main()
