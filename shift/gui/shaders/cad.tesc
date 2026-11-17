#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_control_flow_attributes : require
// Tessellation levels of a CAD patch from the size of its edges on screen
// and from how far they bend away from their chords (sagitta s: n segments
// leave s / n^2 between them and the surface). A level depends only on the
// points of its edge, so patches sharing it agree and leave no cracks.

layout(vertices = 4) out;

layout(location = 0) in vec2 corner_uv[];
layout(location = 1) flat in int patch_index[];
layout(location = 2) in vec4 model_0[];
layout(location = 3) in vec4 model_1[];
layout(location = 4) in vec4 model_2[];
layout(location = 5) in vec4 model_3[];

layout(location = 0) out vec2 patch_uv[];
layout(location = 1) patch out int out_patch;
layout(location = 2) patch out vec4 pose_0;
layout(location = 3) patch out vec4 pose_1;
layout(location = 4) patch out vec4 pose_2;
layout(location = 5) patch out vec4 pose_3;

#include "cad_common.glsl"
#include "cad_uniforms.glsl"
#define FLEX_GRIDS_BINDING 17
#define FLEX_FIELD_BINDING 18
#include "flex_field.glsl"

mat4 model;
CadFace face;
ivec2 span;
int patch_id;
bool deforms; // a flexible body's patch: its points displaced

// the point of the surface at uv in world coordinates, displaced where the
// patch deforms (the surface evaluated once, its displacement added)
vec3 World(vec2 uv) {
  vec3 du, dv;
  vec3 point = SurfacePoint(face, uv, span, du, dv);
  if(deforms) {
    vec3 fu, fv;
    point += FlexDisplacement(patch_id, uv, fu, fv) /
             sample_grids[patch_id].scale;
  }
  return (model * vec4(point, 1.0)).xyz;
}

float Pixels(vec3 a, vec3 b, vec3 at) {
  float distance_to_camera = max(length(at - frame.camera_position.xyz), 1e-6);
  return length(b - a) / distance_to_camera * cad.tessellation.x;
}

float Level(float length_pixels, float sagitta_pixels) {
  float n = max(length_pixels / cad.tessellation.y,
                sqrt(sagitta_pixels / cad.tessellation.w));
  return clamp(n, 1.0, cad.tessellation.z);
}

// the level of an edge from its ends p, q and middle m
float EdgeLevel(vec3 p, vec3 q, vec3 m) {
  return Level(Pixels(p, m, m) + Pixels(m, q, m),
               Pixels(0.5 * (p + q), m, m));
}

void main() {
  patch_uv[gl_InvocationID] = corner_uv[gl_InvocationID];
  if(gl_InvocationID == 0) {
    out_patch = patch_index[0];
    pose_0 = model_0[0];
    pose_1 = model_1[0];
    pose_2 = model_2[0];
    pose_3 = model_3[0];
    model = mat4(model_0[0], model_1[0], model_2[0], model_3[0]);
    patch_id = patch_index[0];
    CadPatch p = patches[patch_id];
    face = faces[p.info.x];
    span = p.info.zw;
    deforms = FlexScale(patch_id) > 0.0;
    // corners: 0 (0, 0), 1 (1, 0), 2 (0, 1), 3 (1, 1). The points of a
    // 3 x 3 grid over the patch, k = 3 j + i at (i/2, j/2), each evaluated
    // once at one place in the shader (World is long; a loop not unrolled
    // keeps one copy of it)
    vec2 lo = corner_uv[0], hi = corner_uv[3];
    vec3 points[9];
    [[dont_unroll]] for(int k = 0; k < 9; k++) {
      vec2 f = 0.5 * vec2(float(k % 3), float(k / 3));
      points[k] = World(mix(lo, hi, f));
    }
    // outer levels of the edges u = 0, v = 0, u = 1, v = 1
    float u0 = EdgeLevel(points[0], points[6], points[3]);
    float v0 = EdgeLevel(points[0], points[2], points[1]);
    float u1 = EdgeLevel(points[2], points[8], points[5]);
    float v1 = EdgeLevel(points[6], points[8], points[7]);
    // the middle may bend more than the edges (e.g. a dome)
    vec3 center = points[4];
    vec3 bilinear =
      0.25 * (points[0] + points[2] + points[6] + points[8]);
    float inner = Level(0.0, Pixels(bilinear, center, center));
    // a flexible body's patch: at least the intervals of its grid of
    // samples, which a flat face bends between
    if(deforms) {
      ivec3 grid = sample_grids[patch_id].grid;
      float nu = min(float(grid.x), cad.tessellation.z);
      float nv = min(float(grid.y), cad.tessellation.z);
      u0 = max(u0, nv);
      u1 = max(u1, nv);
      v0 = max(v0, nu);
      v1 = max(v1, nu);
    }
    gl_TessLevelOuter[0] = u0;
    gl_TessLevelOuter[1] = v0;
    gl_TessLevelOuter[2] = u1;
    gl_TessLevelOuter[3] = v1;
    gl_TessLevelInner[0] = max(0.5 * (v0 + v1), inner);
    gl_TessLevelInner[1] = max(0.5 * (u0 + u1), inner);
  }
}
