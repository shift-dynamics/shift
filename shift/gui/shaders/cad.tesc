#version 450
#extension GL_GOOGLE_include_directive : require
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

mat4 model;
CadFace face;
ivec2 span;

vec3 World(vec2 uv) {
  vec3 du, dv;
  return (model * vec4(SurfacePoint(face, uv, span, du, dv), 1.0)).xyz;
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

float EdgeLevel(vec2 a, vec2 b) {
  vec3 p = World(a);
  vec3 q = World(b);
  vec3 m = World(0.5 * (a + b));
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
    CadPatch p = patches[patch_index[0]];
    face = faces[p.info.x];
    span = p.info.zw;
    // corners: 0 (0, 0), 1 (1, 0), 2 (0, 1), 3 (1, 1); outer levels of the
    // edges u = 0, v = 0, u = 1, v = 1
    float u0 = EdgeLevel(corner_uv[0], corner_uv[2]);
    float v0 = EdgeLevel(corner_uv[0], corner_uv[1]);
    float u1 = EdgeLevel(corner_uv[1], corner_uv[3]);
    float v1 = EdgeLevel(corner_uv[2], corner_uv[3]);
    // the middle may bend more than the edges (e.g. a dome)
    vec2 c = 0.5 * (corner_uv[0] + corner_uv[3]);
    vec3 center = World(c);
    vec3 bilinear = 0.25 * (World(corner_uv[0]) + World(corner_uv[1]) +
                            World(corner_uv[2]) + World(corner_uv[3]));
    float inner = Level(0.0, Pixels(bilinear, center, center));
    gl_TessLevelOuter[0] = u0;
    gl_TessLevelOuter[1] = v0;
    gl_TessLevelOuter[2] = u1;
    gl_TessLevelOuter[3] = v1;
    gl_TessLevelInner[0] = max(0.5 * (v0 + v1), inner);
    gl_TessLevelInner[1] = max(0.5 * (u0 + u1), inner);
  }
}
