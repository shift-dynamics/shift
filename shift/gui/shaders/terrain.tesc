#version 450
#extension GL_GOOGLE_include_directive : require
// Tessellation levels of a terrain patch from the size of its edges on
// screen, after the OpenGL viewer of Shift: an edge is divided so its
// triangles are about tessellation.y pixels wide. A level depends only on
// the two ends of an edge, so patches sharing it agree and leave no cracks.

layout(vertices = 4) out;

layout(location = 0) in vec2 local_xy[];
layout(location = 0) out vec2 patch_xy[];

#include "terrain_common.glsl"

float EdgeLevel(vec2 a, vec2 b) {
  vec3 p = SurfacePoint(a);
  vec3 q = SurfacePoint(b);
  vec3 center = 0.5 * (p + q);
  float distance_to_camera = max(length(center - frame.camera_position.xyz), 1e-3);
  float pixels = length(q - p) / distance_to_camera * terrain.tessellation.x;
  return clamp(pixels / terrain.tessellation.y, 1.0, terrain.tessellation.z);
}

void main() {
  patch_xy[gl_InvocationID] = local_xy[gl_InvocationID];
  if(gl_InvocationID == 0) {
    // corners: 0 (0, 0), 1 (1, 0), 2 (0, 1), 3 (1, 1); outer levels of the
    // edges u = 0, v = 0, u = 1, v = 1
    float u0 = EdgeLevel(local_xy[0], local_xy[2]);
    float v0 = EdgeLevel(local_xy[0], local_xy[1]);
    float u1 = EdgeLevel(local_xy[1], local_xy[3]);
    float v1 = EdgeLevel(local_xy[2], local_xy[3]);
    gl_TessLevelOuter[0] = u0;
    gl_TessLevelOuter[1] = v0;
    gl_TessLevelOuter[2] = u1;
    gl_TessLevelOuter[3] = v1;
    gl_TessLevelInner[0] = 0.5 * (v0 + v1);
    gl_TessLevelInner[1] = 0.5 * (u0 + u1);
  }
}
