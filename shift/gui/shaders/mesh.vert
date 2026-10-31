#version 450

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;

layout(location = 0) out vec3 world_position;
layout(location = 1) out vec3 world_normal;
layout(location = 2) out vec3 body_position;
layout(location = 3) out vec3 body_normal;
layout(location = 4) out vec2 uv;

layout(std140, binding = 0) uniform Frame {
  mat4 view_projection;
  mat4 inverse_view_projection;
  vec4 camera_position;
  vec4 sun_direction; // unit vector towards the sun, w = intensity
  vec4 sun_color;
  vec4 sky_zenith;
  vec4 sky_horizon;
  vec4 floor_params;  // x = floor z, y = grid spacing, z = enabled
} frame;

layout(std140, binding = 1) uniform Instance {
  mat4 model;         // body origin pose, rigid
  vec4 albedo;        // w = normal map scale
  vec4 material;      // x = metallic, y = roughness, z = triplanar,
                      // w = 1 / texture scale
  ivec4 layers;       // base color, metallic roughness, normal; -1: none
} instance;

void main() {
  vec4 world = instance.model * vec4(in_position, 1.0);
  world_position = world.xyz;
  world_normal = mat3(instance.model) * in_normal;
  body_position = in_position;
  body_normal = in_normal;
  uv = in_uv;
  gl_Position = frame.view_projection * world;
}
