#version 330

/* flat.vert */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};

uniform matrices_t matrices_u;

struct material_t {
  vec4 Ka;
  vec4 Kd;
  vec4 Ks;
  vec4 Ke;
  int Ns;
};

uniform material_t material_u;

struct lights_t {
  vec3 position[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
  int directional[10];
  int count;
};

uniform lights_t lights_u;

/* attributes */
layout (location = 0) in vec4 position_a;
layout (location = 1) in vec3 normal_a;

out vec4 vertexGeom;

void main() {
  vertexGeom = position_a;
  gl_Position = matrices_u.mvp * position_a;
}
