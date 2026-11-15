#version 450
#extension GL_GOOGLE_include_directive : require

// pass 1 of the soil surface (fluid_depth.vert): the depth of the sphere of
// a particle along the camera's view direction, the nearest kept by the
// depth test; 0 where there is no particle

layout(location = 0) in vec3 world_position;
layout(location = 1) flat in vec3 center;

layout(location = 0) out vec4 out_depth;

#define FLUID_BINDING 0
#include "fluid_common.glsl"

void main() {
  vec3 o = fluid.camera_position.xyz;
  vec3 d = normalize(world_position - o);
  vec3 oc = o - center;
  float b = dot(oc, d);
  float c = dot(oc, oc) - fluid.particles.x * fluid.particles.x;
  float h = b * b - c;
  if(h < 0.0) {
    discard;
  }
  vec3 p = o + (-b - sqrt(h)) * d;
  out_depth = vec4(dot(p - o, fluid.camera_forward.xyz), 0.0, 0.0, 0.0);
  gl_FragDepth = FluidFragDepth(p);
}
