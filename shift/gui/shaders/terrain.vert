#version 450
// Patches of a terrain: instance k is cell (k % patches_x, k / patches_x) of
// the grid of patches over the terrain, vertex i its corner (i & 1, i >> 1).
// The tessellation shaders place the surface on it (terrain.tesc,
// terrain.tese); mesh.frag shades it like any mesh.

layout(location = 0) out vec2 local_xy; // terrain frame

layout(std140, binding = 6) uniform Terrain {
  mat4 pose;
  vec4 extent; // x, y extent (m), patches along x, y
  vec4 tessellation;
  vec4 scale;
  ivec4 samples;
} terrain;

void main() {
  int patches_x = int(terrain.extent.z);
  ivec2 cell = ivec2(gl_InstanceIndex % patches_x, gl_InstanceIndex / patches_x);
  vec2 corner = vec2(float(gl_VertexIndex & 1), float(gl_VertexIndex >> 1));
  local_xy = (vec2(cell) + corner) / terrain.extent.zw * terrain.extent.xy;
  gl_Position = vec4(local_xy, 0.0, 1.0);
}
