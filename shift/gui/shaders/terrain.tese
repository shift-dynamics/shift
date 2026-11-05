#version 450
#extension GL_GOOGLE_include_directive : require
// The vertices the tessellator generates in a terrain patch, placed on the
// surface evaluated from the control points (the cubic B-spline of the
// physics, terrain_common.glsl) with its normal from the spline's
// derivatives; the outputs are those of mesh.vert, for mesh.frag. The terrain
// is on the fixed body, so its body coordinates are world coordinates.

layout(quads, fractional_odd_spacing, ccw) in;

layout(location = 0) in vec2 patch_xy[];

layout(location = 0) out vec3 world_position;
layout(location = 1) out vec3 world_normal;
layout(location = 2) out vec3 body_position;
layout(location = 3) out vec3 body_normal;
layout(location = 4) out vec2 uv;
layout(location = 5) flat out mat3 body_rotation;

#include "terrain_common.glsl"

void main() {
  vec2 u = gl_TessCoord.xy;
  vec2 xy = mix(mix(patch_xy[0], patch_xy[1], u.x),
                mix(patch_xy[2], patch_xy[3], u.x), u.y);
  vec2 slope;
  float z = LocalZ(xy, slope);
  vec3 world = (terrain.pose * vec4(xy, z, 1.0)).xyz;
  // outward (up, -z): (dz/dx, dz/dy, -1), from the derivatives of the spline
  vec3 normal = normalize(mat3(terrain.pose) * vec3(slope, -1.0));
  world_position = world;
  world_normal = normal;
  body_position = world;
  body_normal = normal;
  uv = xy;
  body_rotation = mat3(1.0);
  gl_Position = frame.view_projection * vec4(world, 1.0);
}
