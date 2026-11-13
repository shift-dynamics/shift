"""Helpers of the example generators that write meshes: triangles with
materials written as an OBJ with its MTL, extruded polygons, cylinders,
spheres, spur gears with involute teeth, and license plates (an image and a
textured quad). Used by
joints/make_screw_and_gear.py and klann_linkage/make_klann_walker.py."""

import math
import os


class Mesh:
    """triangles with materials, written as an OBJ with its MTL"""

    def __init__(self, name, directory):
        self.name = name
        self.directory = directory
        self.vertices = []
        self.normals = []
        self.vertex_index = {}
        self.normal_index = {}
        self.groups = {}
        self.materials = {}

    def material(self, name, color, metal=0.0, rough=0.5):
        self.materials[name] = (color, metal, rough)

    def _vertex(self, p):
        key = tuple(round(x, 6) for x in p)
        if key not in self.vertex_index:
            self.vertices.append(key)
            self.vertex_index[key] = len(self.vertices)
        return self.vertex_index[key]

    def _normal(self, n):
        length = math.sqrt(sum(x * x for x in n)) or 1.0
        key = tuple(round(x / length, 4) for x in n)
        if key not in self.normal_index:
            self.normals.append(key)
            self.normal_index[key] = len(self.normals)
        return self.normal_index[key]

    def triangle(self, material, points, normals):
        """a triangle, wound to agree with its normals"""
        a, b, c = points
        u = [b[i] - a[i] for i in range(3)]
        w = [c[i] - a[i] for i in range(3)]
        geometric = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
                     u[0] * w[1] - u[1] * w[0]]
        if sum(x * x for x in geometric) < 1e-18:
            return
        average = [sum(n[i] for n in normals) for i in range(3)]
        if sum(geometric[i] * average[i] for i in range(3)) < 0:
            points = [points[0], points[2], points[1]]
            normals = [normals[0], normals[2], normals[1]]
        self.groups.setdefault(material, []).append(
            [(self._vertex(p), self._normal(n))
             for p, n in zip(points, normals)])

    def flat(self, material, points):
        """a polygon (convex or a fan from its first point) with one normal"""
        a, b, c = points[0], points[1], points[2]
        u = [b[i] - a[i] for i in range(3)]
        w = [c[i] - a[i] for i in range(3)]
        n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
             u[0] * w[1] - u[1] * w[0]]
        for k in range(1, len(points) - 1):
            self.triangle(material, [points[0], points[k], points[k + 1]],
                          [n, n, n])

    def write(self):
        with open(os.path.join(self.directory, self.name + ".mtl"), "w") as f:
            for name, (color, metal, rough) in self.materials.items():
                f.write(f"newmtl {name}\nKd {color[0]} {color[1]} {color[2]}\n"
                        f"Pm {metal}\nPr {rough}\n\n")
        with open(os.path.join(self.directory, self.name + ".obj"), "w") as f:
            f.write(f"# written by make_screw_and_gear.py\n"
                    f"mtllib {self.name}.mtl\no {self.name}\n")
            for v in self.vertices:
                f.write("v %.6f %.6f %.6f\n" % v)
            for n in self.normals:
                f.write("vn %.4f %.4f %.4f\n" % n)
            for material, faces in self.groups.items():
                f.write(f"usemtl {material}\n")
                for face in faces:
                    f.write("f " + " ".join(f"{v}//{n}" for v, n in face) +
                            "\n")


def prism(mesh, material, outline, z0, z1, hole=None):
    """a polygon (counter-clockwise in x-y, star-shaped about the origin)
    extruded from z0 to z1, with flat walls; hole is the radius of a round
    hole through its middle"""
    count = len(outline)
    for k in range(count):
        (x0, y0), (x1, y1) = outline[k], outline[(k + 1) % count]
        dx, dy = x1 - x0, y1 - y0
        n = (dy, -dx, 0.0)
        mesh.flat(material, [(x0, y0, z0), (x1, y1, z0), (x1, y1, z1),
                             (x0, y0, z1)])
        for z, normal in ((z0, (0, 0, -1)), (z1, (0, 0, 1))):
            a = (x0, y0, z)
            b = (x1, y1, z)
            if hole is None:
                mesh.triangle(material, [(0.0, 0.0, z), a, b], [normal] * 3)
            else:
                ha = (hole * x0 / math.hypot(x0, y0),
                      hole * y0 / math.hypot(x0, y0), z)
                hb = (hole * x1 / math.hypot(x1, y1),
                      hole * y1 / math.hypot(x1, y1), z)
                mesh.triangle(material, [a, b, hb], [normal] * 3)
                mesh.triangle(material, [a, hb, ha], [normal] * 3)
                if z == z0:
                    inward = [(-p[0], -p[1], 0.0) for p in (ha, hb)]
                    mesh.triangle(material, [ha, hb, (hb[0], hb[1], z1)],
                                  [inward[0], inward[1], inward[1]])
                    mesh.triangle(material, [ha, (hb[0], hb[1], z1),
                                             (ha[0], ha[1], z1)],
                                  [inward[0], inward[1], inward[0]])


def cylinder(mesh, material, center, axis, radius, length, sides=28):
    """a cylinder about an axis (a unit vector), smooth sides, flat ends"""
    ax = axis
    helper = (1.0, 0.0, 0.0) if abs(ax[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = [ax[1] * helper[2] - ax[2] * helper[1],
         ax[2] * helper[0] - ax[0] * helper[2],
         ax[0] * helper[1] - ax[1] * helper[0]]
    norm = math.sqrt(sum(x * x for x in u))
    u = [x / norm for x in u]
    v = [ax[1] * u[2] - ax[2] * u[1], ax[2] * u[0] - ax[0] * u[2],
         ax[0] * u[1] - ax[1] * u[0]]

    def ring(k, side):
        t = 2 * math.pi * k / sides
        radial = [math.cos(t) * u[i] + math.sin(t) * v[i] for i in range(3)]
        point = [center[i] + side * 0.5 * length * ax[i] + radius * radial[i]
                 for i in range(3)]
        return point, radial

    for k in range(sides):
        (p0, n0), (p1, n1) = ring(k, -1), ring(k + 1, -1)
        (q0, _), (q1, _) = ring(k, 1), ring(k + 1, 1)
        mesh.triangle(material, [p0, p1, q1], [n0, n1, n1])
        mesh.triangle(material, [p0, q1, q0], [n0, n1, n0])
        for side in (-1, 1):
            mid = [center[i] + side * 0.5 * length * ax[i] for i in range(3)]
            a, b = ring(k, side)[0], ring(k + 1, side)[0]
            mesh.triangle(material, [mid, a, b],
                          [[side * x for x in ax]] * 3)


def sphere(mesh, material, center, radius, rings=10, sides=20):
    def point(i, k):
        phi = math.pi * i / rings
        t = 2 * math.pi * k / sides
        n = (math.sin(phi) * math.cos(t), math.sin(phi) * math.sin(t),
             math.cos(phi))
        return [center[j] + radius * n[j] for j in range(3)], n

    for i in range(rings):
        for k in range(sides):
            a, b = point(i, k), point(i, k + 1)
            c, d = point(i + 1, k + 1), point(i + 1, k)
            mesh.triangle(material, [a[0], b[0], c[0]], [a[1], b[1], c[1]])
            mesh.triangle(material, [a[0], c[0], d[0]], [a[1], c[1], d[1]])




def involute(angle):
    return math.tan(angle) - angle


def gear_outline(teeth, module, tooth_at_zero, pressure_angle=math.radians(20.0),
                 backlash=0.0):
    """outline of a spur gear with involute flanks, counter-clockwise, in
    x-y; pitch radius module * teeth / 2. A tooth (tooth_at_zero) or a gap is
    centered on the x axis; with an even number of teeth the same is true
    of the -x axis. backlash thins each tooth at the pitch circle."""
    pitch_r = module * teeth / 2.0
    base_r = pitch_r * math.cos(pressure_angle)
    tip_r = pitch_r + module
    root_r = pitch_r - 1.25 * module
    half_pitch = math.pi / (2.0 * teeth) - backlash / (2.0 * pitch_r)

    def half_angle(rho):
        return (half_pitch + involute(pressure_angle) -
                involute(math.acos(min(1.0, base_r / rho))))

    steps = 8
    rhos = [base_r + (tip_r - base_r) * k / steps for k in range(steps + 1)]
    phase = 0.0 if tooth_at_zero else math.pi / teeth
    outline = []

    def polar(rho, angle):
        return (rho * math.cos(angle), rho * math.sin(angle))

    for j in range(teeth):
        c = phase + 2.0 * math.pi * j / teeth
        flank = half_angle(base_r)
        outline.append(polar(root_r, c - flank))
        outline += [polar(rho, c - half_angle(rho)) for rho in rhos]
        tip = half_angle(tip_r)
        outline += [polar(tip_r, c + tip * (2 * k / 3 - 1)) for k in range(1, 3)]
        outline += [polar(rho, c + half_angle(rho)) for rho in reversed(rhos)]
        outline.append(polar(root_r, c + flank))
        # the root between this tooth and the next
        start, end = c + flank, c + 2.0 * math.pi / teeth - flank
        outline += [polar(root_r, start + (end - start) * k / 4)
                    for k in range(1, 4)]
    return outline


# strokes of the capital letters of a license plate, in a box one high
# (y down) and LETTER_WIDTH[letter] wide: polylines of points, or arcs as
# ("arc", centre x, centre y, radius x, radius y, from, to) in degrees,
# counterclockwise from +x with y up
PLATE_LETTERS = {
    "S": [("arc", 0.3, 0.255, 0.28, 0.235, 25.0, 270.0),
          ("arc", 0.3, 0.745, 0.3, 0.255, 90.0, -155.0)],
    "H": [[(0.0, 0.0), (0.0, 1.0)], [(0.6, 0.0), (0.6, 1.0)],
          [(0.0, 0.5), (0.6, 0.5)]],
    "I": [[(0.0, 0.0), (0.0, 1.0)]],
    "F": [[(0.0, 1.0), (0.0, 0.0), (0.56, 0.0)], [(0.0, 0.48), (0.44, 0.48)]],
    "T": [[(0.0, 0.0), (0.6, 0.0)], [(0.3, 0.0), (0.3, 1.0)]]}
LETTER_WIDTH = {"S": 0.6, "H": 0.6, "I": 0.0, "F": 0.56, "T": 0.6}


def plate_text(draw, text, left, top, height, color, stroke):
    """draws text (letters of PLATE_LETTERS) with PIL's ImageDraw: its
    strokes (stroke wide, in pixels) inside the box from left, top, height
    pixels high; returns the width drawn"""
    inset = stroke / 2.0
    scale = height - stroke
    x = left
    for letter in text:
        for s in PLATE_LETTERS[letter]:
            if s[0] == "arc":
                _, cx, cy, rx, ry, a0, a1 = s
                n = 40
                points = [(cx + rx * math.cos(math.radians(a0 + (a1 - a0) * k
                                                           / n)),
                           cy - ry * math.sin(math.radians(a0 + (a1 - a0) * k
                                                           / n)))
                          for k in range(n + 1)]
            else:
                points = s
            draw.line([(x + inset + scale * px, top + inset + scale * py)
                       for px, py in points], fill=color, width=int(stroke),
                      joint="curve")
            for px, py in (points[0], points[-1]):  # round ends
                cx, cy = x + inset + scale * px, top + inset + scale * py
                draw.ellipse([cx - stroke / 2, cy - stroke / 2,
                              cx + stroke / 2, cy + stroke / 2], fill=color)
        x += scale * LETTER_WIDTH[letter] + stroke + 0.28 * scale
    return x - 0.28 * scale - left


def license_plate(size, text, band=None, background=(240, 240, 236),
                  ink=(20, 20, 24), border=(30, 30, 34), supersample=4):
    """an image (PIL) of a license plate of size (width, height) pixels: text
    centred in black on white, a thin border, and on the left a band of the
    given color (as on European plates) if band"""
    from PIL import Image, ImageDraw
    w, h = size[0] * supersample, size[1] * supersample
    image = Image.new("RGB", (w, h), border)
    draw = ImageDraw.Draw(image)
    edge = 0.035 * h
    draw.rounded_rectangle([edge, edge, w - edge, h - edge], radius=0.08 * h,
                           fill=background)
    left = edge
    if band:
        draw.rounded_rectangle([edge, edge, edge + 0.11 * w, h - edge],
                               radius=0.08 * h, fill=band)
        draw.rectangle([edge + 0.06 * w, edge, edge + 0.11 * w, h - edge],
                       fill=band)
        left = edge + 0.11 * w
    height = 0.62 * h
    stroke = 0.13 * height
    probe = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    width = plate_text(probe, text, 0, 0, height, ink, stroke)
    room = 0.86 * (w - edge - left)
    if width > room:  # smaller, to fit across
        height *= room / width
        stroke = 0.13 * height
        width = plate_text(probe, text, 0, 0, height, ink, stroke)
    plate_text(draw, text, left + 0.5 * (w - edge - left - width),
               0.5 * (h - height), height, ink, stroke)
    return image.resize(size, Image.LANCZOS)


def textured_quad(directory, name, corners, image_filename):
    """writes name.obj and name.mtl: one quad, with corners bottom left,
    bottom right, top right and top left of the image image_filename (its
    front faces the viewer that sees the image upright)"""
    with open(os.path.join(directory, name + ".mtl"), "w") as f:
        f.write(f"newmtl plate\nKd 1 1 1\nPm 0.0\nPr 0.45\n"
                f"map_Kd {image_filename}\n")
    a, b, c = corners[0], corners[1], corners[2]
    u = [b[i] - a[i] for i in range(3)]
    w = [c[i] - a[i] for i in range(3)]
    n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
         u[0] * w[1] - u[1] * w[0]]
    length = math.sqrt(sum(x * x for x in n))
    with open(os.path.join(directory, name + ".obj"), "w") as f:
        f.write(f"mtllib {name}.mtl\no {name}\n")
        for p in corners:
            f.write("v %.6f %.6f %.6f\n" % tuple(p))
        for t in ((0, 0), (1, 0), (1, 1), (0, 1)):
            f.write("vt %d %d\n" % t)
        f.write("vn %.6f %.6f %.6f\n" % tuple(x / length for x in n))
        f.write("usemtl plate\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")


def textured_mesh(directory, name, positions, uvs, normals, faces,
                  image_filename, roughness=0.45):
    """writes name.obj and name.mtl: triangles faces (index triples) of
    positions, with texture coordinates uvs (u from the left, v from the
    bottom of the image image_filename) and normals, one per position"""
    with open(os.path.join(directory, name + ".mtl"), "w") as f:
        f.write(f"newmtl {name}\nKd 1 1 1\nPm 0.0\nPr {roughness}\n"
                f"map_Kd {image_filename}\n")
    with open(os.path.join(directory, name + ".obj"), "w") as f:
        f.write(f"mtllib {name}.mtl\no {name}\n")
        for p in positions:
            f.write("v %.6f %.6f %.6f\n" % tuple(p))
        for t in uvs:
            f.write("vt %.6f %.6f\n" % tuple(t))
        for n in normals:
            f.write("vn %.6f %.6f %.6f\n" % tuple(n))
        f.write(f"usemtl {name}\n")
        for face in faces:
            f.write("f " + " ".join("%d/%d/%d" % (i + 1, i + 1, i + 1)
                                    for i in face) + "\n")
