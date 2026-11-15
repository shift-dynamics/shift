#version 450

// the particles of an SPH soil (SceneParticles): a sphere mesh, about its
// centre in world units, drawn once per particle at its centre; shaded by
// mesh.frag, with the outputs of mesh.vert

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;
// per instance: the centre of the particle in the world, w unused
layout(location = 3) in vec4 in_center;

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
  ivec4 layers;       // base color, metallic roughness, normal; -1: none;
                      // w = holes used
  vec4 variation;     // as in mesh.frag
  ivec4 surface_layers;
  vec4 surface[2];
  vec4 emission;      // as in mesh.frag
  vec4 holes[4];      // of a terrain, in its uv: x0, y0, x1, y1
} mesh;

void main() {
  vec3 world = in_center.xyz + in_position;
  world_position = world;
  body_rotation = mat3(1.0);
  world_normal = in_normal;
  body_position = in_position;
  body_normal = in_normal;
  uv = in_uv;
  gl_Position = frame.view_projection * vec4(world, 1.0);
}
