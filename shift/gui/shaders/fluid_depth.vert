#version 450
#extension GL_GOOGLE_include_directive : require

// the depth of the soil surface drawn over the particles of an SPH soil,
// pass 1: each particle as a sphere, ray cast in a square that faces the
// camera (fluid_depth.frag)

// per instance: the centre of the particle in the world, w unused
layout(location = 0) in vec4 in_center;

layout(location = 0) out vec3 world_position; // on the square
layout(location = 1) flat out vec3 center;

#define FLUID_BINDING 0
#include "fluid_common.glsl"

void main() {
  const vec2 corners[6] = vec2[](vec2(-1.0, -1.0), vec2(1.0, -1.0),
                                 vec2(-1.0, 1.0), vec2(-1.0, 1.0),
                                 vec2(1.0, -1.0), vec2(1.0, 1.0));
  vec2 corner = corners[gl_VertexIndex];
  float r = fluid.particles.x;
  vec3 to = in_center.xyz - fluid.camera_position.xyz;
  float distance = length(to);
  center = in_center.xyz;
  if(distance <= 1.01 * r) {
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0); // the camera is inside: clipped
    world_position = center;
    return;
  }
  vec3 f = to / distance;
  vec3 up = abs(f.z) < 0.99 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
  vec3 right = normalize(cross(f, up));
  up = cross(right, f);
  // the square through the centre that covers the sphere's silhouette
  float half_width = r * distance / sqrt(distance * distance - r * r);
  world_position = center + half_width * (corner.x * right + corner.y * up);
  gl_Position = fluid.view_projection * vec4(world_position, 1.0);
}
