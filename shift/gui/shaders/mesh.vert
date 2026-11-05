#version 450

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;
// per instance: the body origin pose (rigid), column by column
layout(location = 3) in vec4 in_model_0;
layout(location = 4) in vec4 in_model_1;
layout(location = 5) in vec4 in_model_2;
layout(location = 6) in vec4 in_model_3;

layout(location = 0) out vec3 world_position;
layout(location = 1) out vec3 world_normal;
layout(location = 2) out vec3 body_position;
layout(location = 3) out vec3 body_normal;
layout(location = 4) out vec2 uv;
layout(location = 5) flat out mat3 body_rotation; // body to world

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

layout(std140, binding = 1) uniform Material {
  vec4 albedo;        // w = normal map scale
  vec4 material;      // x = metallic, y = roughness, z = triplanar,
                      // w = 1 / texture scale
  ivec4 layers;       // base color, metallic roughness, normal; -1: none
} mesh;

void main() {
  mat4 model = mat4(in_model_0, in_model_1, in_model_2, in_model_3);
  vec4 world = model * vec4(in_position, 1.0);
  world_position = world.xyz;
  body_rotation = mat3(model);
  world_normal = body_rotation * in_normal;
  body_position = in_position;
  body_normal = in_normal;
  uv = in_uv;
  gl_Position = frame.view_projection * world;
}
