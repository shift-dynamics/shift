#!/usr/bin/env python3
"""Write the meshes of the force module examples, next to this script.

Three unit shapes, each sized in the model files with the mesh "scale":
  box         a cube of side 1, centred on the origin
  cylinder_z  radius 1, length 1 along z, centred on the origin (pulleys)
  tube_x      radius 1, from x = -1 to x = 0 (a beam element, drawn on the
              node at its end)
each written once per material it is drawn in, as <shape>_<material>.obj
with <shape>_<material>.mtl. The materials use the textures built into the
viewer (qrc:/textures/...).

usage: make_meshes.py
"""

import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SEGMENTS = 48

# metallic-roughness materials; colors are linear
STEEL_MAPS = ("map_Kd qrc:/textures/steel_base_color.png\n"
              "map_Pr qrc:/textures/steel_roughness.png\n"
              "norm qrc:/textures/steel_normal.png\n")
PAINT_NORMAL = "norm -bm 0.2 qrc:/textures/steel_normal.png\n"
MATERIALS = {
    "steel": "Kd 0.6 0.6 0.6\nPm 1.0\nPr 1.0\n" + STEEL_MAPS,
    "aluminium": ("Kd 0.91 0.92 0.92\nPm 1.0\nPr 0.35\n"
                  "norm -bm 0.3 qrc:/textures/steel_normal.png\n"),
    "black": "Kd 0.02 0.02 0.02\nPm 0.0\nPr 0.6\n" + PAINT_NORMAL,
    "blue": "Kd 0.02 0.08 0.30\nPm 0.0\nPr 0.35\n" + PAINT_NORMAL,
}


def box():
    """(vertices, normals, uvs, faces) of a unit cube; faces index all
    three lists alike"""
    v, n, t, f = [], [], [], []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            normal = [0.0, 0.0, 0.0]
            normal[axis] = sign
            u_axis, v_axis = [(1, 2), (2, 0), (0, 1)][axis]
            base = len(v)
            for a, b in ((0, 0), (1, 0), (1, 1), (0, 1)):
                p = [0.0, 0.0, 0.0]
                p[axis] = 0.5 * sign
                p[u_axis] = a - 0.5
                p[v_axis] = b - 0.5
                v.append(p)
                n.append(normal)
                t.append((a, b))
            quad = [base, base + 1, base + 2, base + 3]
            if sign < 0:
                quad.reverse()
            f += [(quad[0], quad[1], quad[2]), (quad[0], quad[2], quad[3])]
    return v, n, t, f


def cylinder(axis, start, end):
    """(vertices, normals, uvs, faces) of a closed cylinder of radius 1 from
    start to end along axis (0 = x, 2 = z)"""
    a, b = [(1, 2), (2, 0), (0, 1)][axis]

    def point(angle, s):
        p = [0.0, 0.0, 0.0]
        p[axis] = s
        p[a] = math.cos(angle)
        p[b] = math.sin(angle)
        return p

    v, n, t, f = [], [], [], []
    # side, smooth
    for i in range(SEGMENTS + 1):
        angle = 2 * math.pi * i / SEGMENTS
        radial = point(angle, 0.0)
        for s in (start, end):
            v.append(point(angle, s))
            n.append(radial)
            t.append((i / SEGMENTS, 0.0 if s == start else 1.0))
    for i in range(SEGMENTS):
        p0, p1, p2, p3 = 2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3
        f += [(p0, p2, p3), (p0, p3, p1)]
    # caps, flat
    for s, sign in ((start, -1.0), (end, 1.0)):
        normal = [0.0, 0.0, 0.0]
        normal[axis] = sign
        centre = len(v)
        c = [0.0, 0.0, 0.0]
        c[axis] = s
        v.append(c)
        n.append(normal)
        t.append((0.5, 0.5))
        for i in range(SEGMENTS):
            angle = 2 * math.pi * i / SEGMENTS
            v.append(point(angle, s))
            n.append(normal)
            t.append((0.5 + 0.5 * math.cos(angle), 0.5 + 0.5 * math.sin(angle)))
        for i in range(SEGMENTS):
            j0 = centre + 1 + i
            j1 = centre + 1 + (i + 1) % SEGMENTS
            f.append((centre, j0, j1) if sign > 0 else (centre, j1, j0))
    return v, n, t, f


SHAPES = {
    "box": box(),
    "cylinder_x": cylinder(0, -0.5, 0.5),
    "cylinder_z": cylinder(2, -0.5, 0.5),
    "tube_x": cylinder(0, -1.0, 0.0),
}

# the shapes and materials the examples draw
MESHES = [
    ("box", "black"),          # tip mass of the beam
    ("tube_x", "blue"),        # beam elements
    ("cylinder_z", "black"),   # crankshaft pulley
    ("cylinder_z", "blue"),    # alternator pulley
    ("cylinder_z", "aluminium"),  # water pump pulley
    ("cylinder_z", "steel"),   # backside idler
]

for shape, material in MESHES:
    name = f"{shape}_{material}"
    with open(os.path.join(HERE, name + ".mtl"), "w") as mtl:
        mtl.write(f"# written by make_meshes.py\nnewmtl {material}\n"
                  + MATERIALS[material])
    v, n, t, f = SHAPES[shape]
    with open(os.path.join(HERE, name + ".obj"), "w") as obj:
        obj.write(f"# written by make_meshes.py\nmtllib {name}.mtl\n"
                  f"o {name}\n")
        obj.writelines(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n" for p in v)
        obj.writelines(f"vt {u:.6f} {w:.6f}\n" for u, w in t)
        obj.writelines(f"vn {q[0]:.6f} {q[1]:.6f} {q[2]:.6f}\n" for q in n)
        obj.write(f"usemtl {material}\n")
        obj.writelines(
            "f " + " ".join(f"{i + 1}/{i + 1}/{i + 1}" for i in tri) + "\n"
            for tri in f)
