#version 330

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

in vec4 position_a;
in vec3 normal_a;

out vec3 normalFrag;
out vec4 positionFrag;

void main() {
  normalFrag = normalize(matrices.nm * normal_a);
  positionFrag = position_a;
  gl_Position = matrices.mvp * position_a;
}
