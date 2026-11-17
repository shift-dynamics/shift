// Shared by the DEM shaders that touch convex hulls, after dem_common.glsl
// and the buffers TemplateHull (template_hull), HullVertex (hull_vertex),
// HullPlane (hull_plane) and HullFace (hull_face): the shapes of contact and their overlap, as
// CpuSolver computes them (cpu_solver.cc)

// a shape of contact: a sphere of radius (count 0), a hull's vertices and
// planes at orientation q, centered on the grain's center of mass, or a
// triangle of a body (triangle), its corners from a point of it, with the
// unit normal of its corners in order
struct Shape {
  vec4 q;
  float radius;
  uint first;
  uint count;
  uint first_plane;
  uint planes;
  bool triangle;
  vec3 corner[3];
  vec3 normal;
};

const int kMprIterations = 32;
// the most points of a clipped face: 12 and one more for each of 12 sides
const int kClipped = 32;

// the shape of template sphere k of sphere radius, of a grain at q
Shape ShapeOf(uint k, float radius, vec4 q) {
  const uvec4 h = template_hull[k];
  Shape s;
  s.q = q;
  s.radius = h.y > 0u ? hull_vertex[h.x].w : radius;
  s.first = h.x;
  s.count = h.y;
  s.first_plane = h.z;
  s.planes = h.w;
  s.triangle = false;
  s.corner = vec3[3](vec3(0.0), vec3(0.0), vec3(0.0));
  s.normal = vec3(0.0);
  return s;
}

// the point of the shape farthest along d (from its center)
vec3 Support(Shape s, vec3 d) {
  if(s.count == 0u) {
    return d * (s.radius / length(d));
  }
  if(s.triangle) {
    vec3 best = s.corner[0];
    for(int k = 1; k < 3; k++) {
      if(dot(d, s.corner[k]) > dot(d, best)) {
        best = s.corner[k];
      }
    }
    return best;
  }
  const vec3 local = RotateBack(s.q, d);
  uint best = s.first;
  float most = dot(local, hull_vertex[best].xyz);
  for(uint k = s.first + 1u; k < s.first + s.count; k++) {
    const float h = dot(local, hull_vertex[k].xyz);
    if(h > most) {
      most = h;
      best = k;
    }
  }
  return Rotate(s.q, hull_vertex[best].xyz);
}

// the face of hull s most nearly along the unit d (from its center): its
// plane, and how nearly (the cosine). A triangle has two faces, 0 along its
// normal and 1 against it
uint FaceAlong(Shape s, vec3 d, out float alignment) {
  if(s.triangle) {
    const float a = dot(d, s.normal);
    alignment = abs(a);
    return a >= 0.0 ? 0u : 1u;
  }
  const vec3 local = RotateBack(s.q, d);
  uint best = s.first_plane;
  alignment = dot(local, hull_plane[best].xyz);
  for(uint k = s.first_plane + 1u; k < s.first_plane + s.planes; k++) {
    const float a = dot(local, hull_plane[k].xyz);
    if(a > alignment) {
      alignment = a;
      best = k;
    }
  }
  return best;
}

// the outward unit normal of face f of s
vec3 FaceNormal(Shape s, uint f) {
  if(s.triangle) {
    return f == 0u ? s.normal : -s.normal;
  }
  return Rotate(s.q, hull_plane[f].xyz);
}

// the sides of face f of s
uint FaceSides(Shape s, uint f) {
  return s.triangle ? 3u : hull_face[f].x;
}

// vertex k of face f of s, counterclockwise about its outward normal, from
// the shape's center at c
vec3 FaceVertex(Shape s, uint f, uint k, vec3 c) {
  if(s.triangle) {
    return c + s.corner[f == 0u ? k : (3u - k) % 3u];
  }
  const uint index = (hull_face[f][1u + k / 4u] >> (8u * (k % 4u))) & 0xffu;
  return c + Rotate(s.q, hull_vertex[s.first + index].xyz);
}

vec3 clipped[2][kClipped];

// the point of contact of hulls a at c and b at 0 touching along n (from b
// to a), the spread of the patch and the weight of the point (see
// ClippedPoint in cpu_solver.cc)
bool ClippedPoint(Shape a, Shape b, vec3 c, vec3 n, out vec3 point,
                  out float spread, out float weight) {
  point = vec3(0.0);
  spread = 0.0;
  weight = 0.0;
  float align_a, align_b;
  const uint fa = FaceAlong(a, -n, align_a), fb = FaceAlong(b, n, align_b);
  const bool reference_b = align_b >= align_a;
  const Shape r = reference_b ? b : a;
  const Shape i = reference_b ? a : b;
  const vec3 cr = reference_b ? vec3(0.0) : c, ci = reference_b ? c : vec3(0.0);
  const uint fr = reference_b ? fb : fa, fi = reference_b ? fa : fb;
  int count = int(FaceSides(i, fi)), current = 0;
  for(int k = 0; k < count; k++) {
    clipped[0][k] = FaceVertex(i, fi, uint(k), ci);
  }
  const vec3 normal = FaceNormal(r, fr);
  const uint sides = FaceSides(r, fr);
  for(uint e = 0u; e < sides && count > 0; e++) {
    const vec3 p = FaceVertex(r, fr, e, cr);
    const vec3 q = FaceVertex(r, fr, (e + 1u) % sides, cr);
    const vec3 side = cross(q - p, normal);
    int next = 0;
    for(int k = 0; k < count; k++) {
      const vec3 u = clipped[current][k];
      const vec3 v = clipped[current][(k + 1) % count];
      const float du = dot(side, u - p), dv = dot(side, v - p);
      if(du <= 0.0 && next < kClipped) {
        clipped[1 - current][next++] = u;
      }
      if((du <= 0.0) != (dv <= 0.0) && next < kClipped) {
        clipped[1 - current][next++] = u + (du / (du - dv)) * (v - u);
      }
    }
    current = 1 - current;
    count = next;
  }
  const vec3 base = FaceVertex(r, fr, 0u, cr);
  vec3 sum = vec3(0.0);
  float total = 0.0;
  for(int k = 0; k < count; k++) {
    const vec3 x = clipped[current][k];
    const float depth = dot(normal, base - x);
    if(depth > 0.0) {
      sum = sum + depth * (x + (0.5 * depth) * normal);
      total += depth;
    }
  }
  if(!(total > 0.0)) {
    return false;
  }
  point = sum / total;
  weight = total;
  for(int k = 0; k < count; k++) {
    const vec3 x = clipped[current][k];
    const float depth = dot(normal, base - x);
    if(depth > 0.0) {
      const vec3 arm = x - point;
      const vec3 tangent = arm - dot(arm, normal) * normal;
      spread += depth * dot(tangent, tangent);
    }
  }
  spread = spread / total;
  return true;
}

// the support of the Minkowski difference of a at c less b at 0
vec3 Minkowski(Shape a, Shape b, vec3 c, vec3 d) {
  return (c + Support(a, d)) - Support(b, -d);
}

// v / |v|, false if |v| is not above tiny
bool Unit(vec3 v, float tiny, out vec3 u) {
  const float size = length(v);
  u = vec3(0.0);
  if(!(size > tiny)) {
    return false;
  }
  u = v / size;
  return true;
}

// whether shape a, centered at c from b's center, overlaps b; then the
// depth, the unit normal from b to a, the point of contact from b's
// center, the spread of the patch and the weight of the point (Minkowski
// portal refinement; see Collide in cpu_solver.cc)
bool Collide(Shape a, Shape b, vec3 c, float scale, out vec3 n,
             out float depth, out vec3 point, out float spread,
             out float weight) {
  spread = 0.0;
  weight = 0.0;
  n = vec3(0.0);
  depth = 0.0;
  point = vec3(0.0);
  const float tiny = 1e-12 * scale, tolerance = 1e-6 * scale;
  vec3 v0 = c;
  if(!(length(v0) > tiny)) {
    v0 = vec3(tolerance, 0.0, 0.0);
  }
  vec3 dir;
  if(!Unit(-v0, 0.0, dir)) {
    return false;
  }
  vec3 v1 = Minkowski(a, b, c, dir);
  if(!(dot(v1, dir) > 0.0)) {
    return false;
  }
  if(!Unit(cross(v0, v1), tiny * scale, dir)) {
    depth = length(v1);
    if(!Unit(v0, 0.0, n)) {
      return false;
    }
    point = 0.5 * ((c + Support(a, -n)) + Support(b, n));
    weight = depth;
    return depth > 0.0;
  }
  vec3 v2 = Minkowski(a, b, c, dir);
  if(!(dot(v2, dir) > 0.0)) {
    return false;
  }
  if(!Unit(cross(v1 - v0, v2 - v0), tiny * scale, dir)) {
    return false;
  }
  if(dot(dir, v0) > 0.0) {
    const vec3 t = v1;
    v1 = v2;
    v2 = t;
    dir = -dir;
  }
  vec3 v3;
  int k = 0;
  for(; k < kMprIterations; k++) {
    v3 = Minkowski(a, b, c, dir);
    if(!(dot(v3, dir) > 0.0)) {
      return false;
    }
    if(dot(cross(v1, v3), v0) < 0.0) {
      v2 = v3;
      if(!Unit(cross(v1 - v0, v3 - v0), tiny * scale, dir)) {
        return false;
      }
      continue;
    }
    if(dot(cross(v3, v2), v0) < 0.0) {
      v1 = v3;
      if(!Unit(cross(v3 - v0, v2 - v0), tiny * scale, dir)) {
        return false;
      }
      continue;
    }
    break;
  }
  if(k == kMprIterations) {
    return false;
  }
  for(k = 0; k < kMprIterations; k++) {
    if(!Unit(cross(v2 - v1, v3 - v1), tiny * scale, dir)) {
      return false;
    }
    const vec3 v4 = Minkowski(a, b, c, dir);
    const float d4 = dot(v4, dir);
    if(!(d4 > 0.0)) {
      return false;
    }
    const float gap = min(d4 - dot(v1, dir),
                          min(d4 - dot(v2, dir), d4 - dot(v3, dir)));
    if(gap <= tolerance) {
      break;
    }
    const vec3 x = cross(v4, v0);
    if(dot(v1, x) > 0.0) {
      if(dot(v2, x) > 0.0) {
        v1 = v4;
      } else {
        v3 = v4;
      }
    } else {
      if(dot(v3, x) > 0.0) {
        v2 = v4;
      } else {
        v1 = v4;
      }
    }
  }
  depth = dot(v1, dir);
  if(!(depth > 0.0)) {
    return false;
  }
  n = -dir;
  if(a.count == 0u || b.count == 0u ||
     !ClippedPoint(a, b, c, n, point, spread, weight)) {
    spread = 0.0;
    point = 0.5 * ((c + Support(a, -n)) + Support(b, n));
    weight = depth;
  }
  return true;
}
