#version 450
#extension GL_GOOGLE_include_directive : require
// The vertices the tessellator generates in a CAD patch, placed on the exact
// surface of its face with the normal from the surface's derivatives (the
// face's normal, reversed where the face is against its surface); the
// outputs are those of mesh.vert and the face, for mesh.frag compiled with
// CAD, which tests the trims.

layout(quads, fractional_odd_spacing, ccw) in;

layout(location = 0) in vec2 patch_uv[];
layout(location = 1) patch in int out_patch;
layout(location = 2) patch in vec4 pose_0;
layout(location = 3) patch in vec4 pose_1;
layout(location = 4) patch in vec4 pose_2;
layout(location = 5) patch in vec4 pose_3;

layout(location = 0) out vec3 world_position;
layout(location = 1) out vec3 world_normal;
layout(location = 2) out vec3 body_position;
layout(location = 3) out vec3 body_normal;
layout(location = 4) out vec2 uv;
layout(location = 5) flat out mat3 body_rotation;
layout(location = 8) flat out ivec2 cad_face; // face, flags

#include "cad_common.glsl"
#include "cad_uniforms.glsl"

// a + (b - a) t, exactly a and b at the ends, so that patches sharing an
// edge place its points at exactly the same (u, v)
float Lerp(float a, float b, float t) {
  return t <= 0.0 ? a : t >= 1.0 ? b : a + (b - a) * t;
}

void main() {
  vec2 t = gl_TessCoord.xy;
  // corners 0 (u0, v0) and 3 (u1, v1) of the patch's rectangle
  vec2 st = vec2(Lerp(patch_uv[0].x, patch_uv[3].x, t.x),
                 Lerp(patch_uv[0].y, patch_uv[3].y, t.y));
  CadPatch p = patches[out_patch];
  vec3 n;
  vec3 point = PatchPoint(p, st, n);
  mat4 model = mat4(pose_0, pose_1, pose_2, pose_3);
  vec4 world = model * vec4(point, 1.0);
  world_position = world.xyz;
  body_rotation = mat3(model);
  world_normal = normalize(body_rotation * n);
  body_position = point;
  body_normal = n;
  uv = st;
  cad_face = p.info.xy;
  gl_Position = frame.view_projection * world;
}
