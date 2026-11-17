// the displacement of a flexible body's CAD surface (SceneCad::flex_mesh):
// per patch a grid of samples (SceneCad::sample_grids), their displacements
// written each frame by flex_field.comp, interpolated Catmull-Rom over the
// grid as shift::gui::FlexCadInterpolate does on the CPU. The bindings are
// set before including it.

struct CadSampleGrid {
  vec4 uv;     // the patch's u0, v0, u1, v1
  ivec3 grid;  // nu, nv (intervals), first sample
  float scale; // body coordinates per CAD unit; 0: the patch does not deform
};

layout(std430, binding = FLEX_GRIDS_BINDING) readonly buffer FlexGrids {
  CadSampleGrid sample_grids[];
};
layout(std430, binding = FLEX_FIELD_BINDING) readonly buffer FlexField {
  vec4 field[];
};

// the scale of patch's samples, 0 if it does not deform (a CAD that does not
// deform binds a single grid of scale 0)
float FlexScale(int patch_index) {
  return patch_index < sample_grids.length()
           ? sample_grids[patch_index].scale
           : 0.0;
}

// Catmull-Rom weights of the samples before, at, after and two after t in
// [0, 1], and their derivatives along t
void CatmullRom(float t, out vec4 w, out vec4 dw) {
  float t2 = t * t, t3 = t2 * t;
  w = 0.5 * vec4(-t + 2.0 * t2 - t3, 2.0 - 5.0 * t2 + 3.0 * t3,
                 t + 4.0 * t2 - 3.0 * t3, -t2 + t3);
  dw = 0.5 * vec4(-1.0 + 4.0 * t - 3.0 * t2, -10.0 * t + 9.0 * t2,
                  1.0 + 8.0 * t - 9.0 * t2, -2.0 * t + 3.0 * t2);
}

vec3 FlexSample(CadSampleGrid g, int i, int j) {
  return field[g.grid.z + j * (g.grid.x + 1) + i].xyz;
}

// a sample along the line of v index j, the grid extended linearly beyond
// its ends along u (one beyond: 2 s_0 - s_1, 2 s_n - s_(n-1)); without
// branches, which the shader compilers would otherwise duplicate the loads
// of the samples for
vec3 FlexAlongU(CadSampleGrid g, int i, int j) {
  int c = clamp(i, 0, g.grid.x);
  int e = i - c; // -1, 0 or 1
  vec3 at = FlexSample(g, c, j);
  return at + float(abs(e)) * (at - FlexSample(g, c - e, j));
}

// and beyond its ends along v
vec3 FlexAt(CadSampleGrid g, int i, int j) {
  int c = clamp(j, 0, g.grid.y);
  int e = j - c;
  vec3 at = FlexAlongU(g, i, c);
  return at + float(abs(e)) * (at - FlexAlongU(g, i, c - e));
}

// the displacement (body coordinates) at uv of a patch, and its derivatives
// along u and v
vec3 FlexDisplacement(int patch_index, vec2 uv, out vec3 along_u,
                      out vec3 along_v) {
  CadSampleGrid g = sample_grids[patch_index];
  vec2 n = vec2(g.grid.xy);
  vec2 s = clamp((uv - g.uv.xy) / (g.uv.zw - g.uv.xy) * n, vec2(0.0), n);
  ivec2 i = min(ivec2(floor(s)), g.grid.xy - 1);
  vec4 wu, dwu, wv, dwv;
  CatmullRom(s.x - float(i.x), wu, dwu);
  CatmullRom(s.y - float(i.y), wv, dwv);
  vec3 x = vec3(0.0), xu = vec3(0.0), xv = vec3(0.0);
  // not unrolled: the samples' loads stay one copy
  [[dont_unroll]] for(int q = 0; q < 4; q++) {
    [[dont_unroll]] for(int p = 0; p < 4; p++) {
      vec3 value = FlexAt(g, i.x - 1 + p, i.y - 1 + q);
      x += wu[p] * wv[q] * value;
      xu += dwu[p] * wv[q] * value;
      xv += wu[p] * dwv[q] * value;
    }
  }
  along_u = xu * n.x / (g.uv.z - g.uv.x);
  along_v = xv * n.y / (g.uv.w - g.uv.y);
  return x;
}

// the point of a patch's face at uv in CAD units, and its unit normal
// (PatchPoint), displaced by the field where the patch deforms
vec3 DeformedPatchPoint(int patch_index, vec2 uv, out vec3 normal) {
  float scale = FlexScale(patch_index);
  CadPatch p = patches[patch_index];
  CadFace f = faces[p.info.x];
  // the surface evaluated once, as PatchPoint does: the shader compilers
  // inline each call
  vec3 du, dv;
  vec3 point = SurfacePoint(f, uv, p.info.zw, du, dv);
  if(scale > 0.0) {
    vec3 fu, fv;
    point += FlexDisplacement(patch_index, uv, fu, fv) / scale;
    du += fu / scale;
    dv += fv / scale;
  }
  vec3 n = cross(du, dv);
  float size = dot(du, du) + dot(dv, dv);
  if(dot(n, n) <= 1e-12 * size * size) {
    // a singular point (apex, pole): the normal a little way in, undeformed
    vec2 center = 0.5 * (p.uv.xy + p.uv.zw);
    vec3 a, b;
    SurfacePoint(f, mix(uv, center, 1e-3), p.info.zw, a, b);
    n = cross(a, b);
  }
  normal = normalize(n) * f.extrusion.w;
  return point;
}
