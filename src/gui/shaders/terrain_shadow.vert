#version 420 core

/* terrain_shadow.vert */
layout (binding = 1) uniform sampler2D texture_elevation_u;

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat4 p;
};

uniform matrices_t matrices_u;

struct terrain_t {
  float zMin;
  float zMax;
  vec2 scale;
  vec2 viewport;
};

uniform terrain_t terrain_u;

out VS_OUT {
  vec2 tc;
} vs_out;

void main(void) {
  const vec4 vertices[] = vec4[](
    vec4(0.0, 0.0, 0.0, 1.0),
    vec4(1.0, 0.0, 0.0, 1.0),
    vec4(0.0, 1.0, 0.0, 1.0),
    vec4(1.0, 1.0, 0.0, 1.0));

  ivec2 texture_size = textureSize(texture_elevation_u, 0);

  int x = gl_InstanceID % texture_size.x; // row index into elevation map
  int y = gl_InstanceID / texture_size.x; // column index into elevation map

  vs_out.tc = (vertices[gl_VertexID].xy + vec2(x, y)) / texture_size;

  gl_Position =
    vertices[gl_VertexID] +
    vec4(
      float(x),
      float(y),
      0.0,
      0.0);
}
