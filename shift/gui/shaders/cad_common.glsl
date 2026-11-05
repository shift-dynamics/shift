// CAD geometry (cad_scene.h): the records of faces, swept curves and
// patches, and the evaluation of the exact surfaces of faces, as
// shift::core::cad::SurfacePoint does on the CPU. Shared by cad.vert,
// cad.tesc, cad.tese, cad_triangulate.comp, mesh.frag compiled with CAD and
// raytrace.comp. The bindings may be set before including it.

#ifndef CAD_FACES_BINDING
#define CAD_FACES_BINDING 10
#endif
#ifndef CAD_CURVES_BINDING
#define CAD_CURVES_BINDING 11
#endif
#ifndef CAD_PATCHES_BINDING
#define CAD_PATCHES_BINDING 12
#endif
#ifndef CAD_POINTS_BINDING
#define CAD_POINTS_BINDING 13
#endif
#ifndef CAD_KNOTS_BINDING
#define CAD_KNOTS_BINDING 14
#endif
#ifndef CAD_SEGMENTS_BINDING
#define CAD_SEGMENTS_BINDING 15
#endif
#ifndef CAD_TRIM_INDEX_BINDING
#define CAD_TRIM_INDEX_BINDING 16
#endif

struct CadFace {
  vec4 origin;    // w: surface type
  vec4 axis_x;    // w: radius
  vec4 axis_y;    // w: minor radius
  vec4 axis_z;    // w: tan(semi angle)
  vec4 extrusion; // w: +1, or -1 if the face's normal is the surface's
                  // reversed
  vec4 color;     // w: 1 if the face has its own color
  vec4 trim_box;  // x: v of the first band, y: band height
  ivec4 nurbs;    // u degree, v degree, u count, v count
  ivec4 offsets;  // first u knot, first point, swept curve
  ivec4 trim;     // first band header, band count
};

struct CadCurve {
  vec4 origin; // w: type (0 line, 1 conic, 2 NURBS)
  vec4 a;      // line: direction
  vec4 b;
  ivec4 nurbs; // degree, count, first knot, first point
};

struct CadPatch {
  vec4 uv;    // u0, v0, u1, v1
  ivec4 info; // face, flags (1: crosses the trims), u span, v span
};

layout(std430, binding = CAD_FACES_BINDING) readonly buffer CadFaces {
  CadFace faces[];
};

#ifndef CAD_TRIM_ONLY
layout(std430, binding = CAD_CURVES_BINDING) readonly buffer CadCurves {
  CadCurve curves[];
};
layout(std430, binding = CAD_PATCHES_BINDING) readonly buffer CadPatches {
  CadPatch patches[];
};
layout(std430, binding = CAD_POINTS_BINDING) readonly buffer CadPoints {
  vec4 points[];
};
layout(std430, binding = CAD_KNOTS_BINDING) readonly buffer CadKnots {
  float knots[];
};

const int kPlane = 0;
const int kCylinder = 1;
const int kCone = 2;
const int kSphere = 3;
const int kTorus = 4;
const int kNurbs = 5;
const int kRevolution = 6;
const int kExtrusion = 7;

const int kMaxDegree = 15;

// the B-spline basis functions of degree p non-zero in span s (the knots
// from k0), and their derivatives, at t (Piegl and Tiller A2.2, and the
// derivative from the functions of degree p - 1)
void Basis(int k0, int p, int s, float t, out float n[kMaxDegree + 1],
           out float dn[kMaxDegree + 1]) {
  float left[kMaxDegree + 1];
  float right[kMaxDegree + 1];
  float lower[kMaxDegree + 1];
  n[0] = 1.0;
  for(int j = 1; j <= p; j++) {
    if(j == p) {
      for(int r = 0; r < p; r++) {
        lower[r] = n[r];
      }
    }
    left[j] = t - knots[k0 + s + 1 - j];
    right[j] = knots[k0 + s + j] - t;
    float saved = 0.0;
    for(int r = 0; r < j; r++) {
      float den = right[r + 1] + left[j - r];
      float temp = den != 0.0 ? n[r] / den : 0.0;
      n[r] = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    n[j] = saved;
  }
  for(int r = 0; r <= p; r++) {
    float d = 0.0;
    if(p > 0) {
      int i = s - p + r;
      if(r >= 1) {
        float den = knots[k0 + i + p] - knots[k0 + i];
        d += den != 0.0 ? lower[r - 1] / den : 0.0;
      }
      if(r <= p - 1) {
        float den = knots[k0 + i + p + 1] - knots[k0 + i + 1];
        d -= den != 0.0 ? lower[r] / den : 0.0;
      }
    }
    dn[r] = float(p) * d;
  }
}

// a swept curve at t and its derivative
vec3 CurvePoint(int index, int span, float t, out vec3 derivative) {
  CadCurve c = curves[index];
  int type = int(c.origin.w);
  if(type == 0) {
    derivative = c.a.xyz;
    return c.origin.xyz + t * c.a.xyz;
  }
  if(type == 1) {
    float ct = cos(t), st = sin(t);
    derivative = -st * c.a.xyz + ct * c.b.xyz;
    return c.origin.xyz + ct * c.a.xyz + st * c.b.xyz;
  }
  int p = min(c.nurbs.x, kMaxDegree);
  float n[kMaxDegree + 1];
  float dn[kMaxDegree + 1];
  Basis(c.nurbs.z, p, span, t, n, dn);
  vec4 a = vec4(0.0), da = vec4(0.0);
  for(int r = 0; r <= p; r++) {
    vec4 q = points[c.nurbs.w + span - p + r];
    vec4 h = vec4(q.xyz * q.w, q.w);
    a += n[r] * h;
    da += dn[r] * h;
  }
  vec3 point = a.xyz / a.w;
  derivative = (da.xyz - da.w * point) / a.w;
  return point;
}

// v turned by the angle u about the unit axis z
vec3 Turn(vec3 v, vec3 z, float u) {
  vec3 along = dot(v, z) * z;
  vec3 perp = v - along;
  return along + cos(u) * perp + sin(u) * cross(z, perp);
}

// the point of a face's surface at uv (body coordinates, before the scale)
// and its derivatives; span: the knot spans of the patch
vec3 SurfacePoint(CadFace f, vec2 uv, ivec2 span, out vec3 du, out vec3 dv) {
  int type = int(f.origin.w);
  vec3 o = f.origin.xyz, x = f.axis_x.xyz, y = f.axis_y.xyz, z = f.axis_z.xyz;
  float cu = cos(uv.x), su = sin(uv.x);
  if(type == kPlane) {
    du = x;
    dv = y;
    return o + uv.x * x + uv.y * y;
  }
  if(type == kCylinder) {
    float r = f.axis_x.w;
    du = r * (-su * x + cu * y);
    dv = z;
    return o + r * (cu * x + su * y) + uv.y * z;
  }
  if(type == kCone) {
    float t = f.axis_z.w;
    float r = f.axis_x.w + uv.y * t;
    du = r * (-su * x + cu * y);
    dv = t * (cu * x + su * y) + z;
    return o + r * (cu * x + su * y) + uv.y * z;
  }
  if(type == kSphere) {
    float R = f.axis_x.w;
    float cv = cos(uv.y), sv = sin(uv.y);
    du = R * cv * (-su * x + cu * y);
    dv = R * (-sv * (cu * x + su * y) + cv * z);
    return o + R * cv * (cu * x + su * y) + R * sv * z;
  }
  if(type == kTorus) {
    float rm = f.axis_y.w;
    float cv = cos(uv.y), sv = sin(uv.y);
    float r = f.axis_x.w + rm * cv;
    du = r * (-su * x + cu * y);
    dv = rm * (-sv * (cu * x + su * y) + cv * z);
    return o + r * (cu * x + su * y) + rm * sv * z;
  }
  if(type == kRevolution) {
    vec3 dc;
    vec3 q = CurvePoint(f.offsets.z, span.y, uv.y, dc);
    vec3 p = Turn(q - o, z, uv.x);
    du = cross(z, p);
    dv = Turn(dc, z, uv.x);
    return o + p;
  }
  if(type == kExtrusion) {
    vec3 dc;
    vec3 q = CurvePoint(f.offsets.z, span.x, uv.x, dc);
    du = dc;
    dv = f.extrusion.xyz;
    return q + uv.y * f.extrusion.xyz;
  }
  // NURBS: tensor product of the basis functions of the patch's spans
  int pu = min(f.nurbs.x, kMaxDegree), pv = min(f.nurbs.y, kMaxDegree);
  int v_count = f.nurbs.w;
  float nu[kMaxDegree + 1], dnu[kMaxDegree + 1];
  float nv[kMaxDegree + 1], dnv[kMaxDegree + 1];
  Basis(f.offsets.x, pu, span.x, uv.x, nu, dnu);
  Basis(f.offsets.x + f.nurbs.z + pu + 1, pv, span.y, uv.y, nv, dnv);
  vec4 a = vec4(0.0), au = vec4(0.0), av = vec4(0.0);
  for(int r = 0; r <= pu; r++) {
    int i = span.x - pu + r;
    for(int q = 0; q <= pv; q++) {
      int j = span.y - pv + q;
      vec4 c = points[f.offsets.y + i * v_count + j];
      vec4 h = vec4(c.xyz * c.w, c.w);
      a += nu[r] * nv[q] * h;
      au += dnu[r] * nv[q] * h;
      av += nu[r] * dnv[q] * h;
    }
  }
  vec3 point = a.xyz / a.w;
  du = (au.xyz - au.w * point) / a.w;
  dv = (av.xyz - av.w * point) / a.w;
  return point;
}

// the point of a patch's face at uv and the face's unit normal (the
// surface's, reversed where the face is against it); at a singular point
// (apex, pole) the normal a little way into the patch
vec3 PatchPoint(CadPatch p, vec2 uv, out vec3 normal) {
  CadFace f = faces[p.info.x];
  vec3 du, dv;
  vec3 point = SurfacePoint(f, uv, p.info.zw, du, dv);
  vec3 n = cross(du, dv);
  float scale = dot(du, du) + dot(dv, dv);
  if(dot(n, n) <= 1e-12 * scale * scale) {
    vec2 center = 0.5 * (p.uv.xy + p.uv.zw);
    vec3 a, b;
    SurfacePoint(f, mix(uv, center, 1e-3), p.info.zw, a, b);
    n = cross(a, b);
  }
  normal = normalize(n) * f.extrusion.w;
  return point;
}
#endif

#ifdef CAD_TRIM
layout(std430, binding = CAD_SEGMENTS_BINDING) readonly buffer CadSegments {
  vec4 segments[];
};
layout(std430, binding = CAD_TRIM_INDEX_BINDING) readonly buffer CadTrimIndex {
  int trim_index[];
};

// whether uv is inside the trims of a face (nonzero winding number of its
// segments), as CadInside on the CPU
bool InsideTrims(int face, vec2 uv) {
  CadFace f = faces[face];
  int band = clamp(int(floor((uv.y - f.trim_box.x) / f.trim_box.y)), 0,
                   f.trim.y - 1);
  int first = trim_index[f.trim.x + 2 * band];
  int count = trim_index[f.trim.x + 2 * band + 1];
  int winding = 0;
  for(int k = 0; k < count; k++) {
    int index = trim_index[first + k];
    vec4 s = segments[index < 0 ? ~index : index];
    if((s.y <= uv.y) != (s.w <= uv.y)) {
      float x = s.x + (uv.y - s.y) * (s.z - s.x) / (s.w - s.y);
      if(x > uv.x) {
        winding += s.w > s.y ? 1 : -1;
      }
    }
  }
  return winding != 0;
}

// whether uv is inside the trims of a face (nonzero winding number of its
// segments), its distance in pixels from them, and from those drawn with
// edge lines (not seams or singular points), given the change of uv per
// pixel along x and y of the screen
bool InsideTrims(int face, vec2 uv, vec2 duv_dx, vec2 duv_dy,
                 out float pixels, out float edge_pixels) {
  CadFace f = faces[face];
  int band = clamp(int(floor((uv.y - f.trim_box.x) / f.trim_box.y)), 0,
                   f.trim.y - 1);
  int first = trim_index[f.trim.x + 2 * band];
  int count = trim_index[f.trim.x + 2 * band + 1];
  // pixels per unit of u and v
  mat2 to_uv = mat2(duv_dx, duv_dy);
  float det = determinant(to_uv);
  mat2 to_pixels = abs(det) > 1e-30 ? inverse(to_uv) : mat2(1e30);
  int winding = 0;
  pixels = 1e30;
  edge_pixels = 1e30;
  for(int k = 0; k < count; k++) {
    // ~index (negative): a segment without an edge line
    int index = trim_index[first + k];
    vec4 s = segments[index < 0 ? ~index : index];
    vec2 a = s.xy, b = s.zw;
    if((a.y <= uv.y) != (b.y <= uv.y)) {
      float x = a.x + (uv.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if(x > uv.x) {
        winding += b.y > a.y ? 1 : -1;
      }
    }
    // nearest point of the segment, measured on screen
    vec2 e = b - a;
    vec2 ea = to_pixels * e, pa = to_pixels * (uv - a);
    float t = clamp(dot(pa, ea) / max(dot(ea, ea), 1e-30), 0.0, 1.0);
    float d = length(pa - t * ea);
    pixels = min(pixels, d);
    if(index >= 0) {
      edge_pixels = min(edge_pixels, d);
    }
  }
  return winding != 0;
}
#endif
