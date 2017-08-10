#version 420 core

uniform sampler2D tex_displacement;

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};

uniform matrices_t matrices;

struct material_t {
  vec4 Ka;
  vec4 Kd;
  vec4 Ks;
  vec4 Ke;
  int Ns;
};

uniform material_t material;

struct lights_t {
  vec3 position[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int directional[10];
  int count;
};

uniform lights_t lights;

struct terrain_t {
  float zMin;
  float zMax;
  vec2 viewport;
};

uniform terrain_t terrain;

out VS_OUT {
  vec2 tc;
} vs_out;

void main(void) {
  const vec4 vertices[] = vec4[](
    vec4(0.0, 0.0, 0.0, 1.0),
    vec4(1.0, 0.0, 0.0, 1.0),
    vec4(0.0, 1.0, 0.0, 1.0),
    vec4(1.0, 1.0, 0.0, 1.0));
  int x = gl_InstanceID & 63; // grid width
  int y = gl_InstanceID >> 6; // grid height
  vec2 offs = vec2(x, y);
  vs_out.tc = (vertices[gl_VertexID].xy + offs) / 64.0;
  gl_Position =
    vertices[gl_VertexID] +
    vec4(float(x), float(y), 0.0, 0.0);
}
