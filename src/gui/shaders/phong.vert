#version 330

uniform sampler2DShadow texture_shadow_u[10];

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
  int drawShadows;
  mat4 shadowMatrix[10];
};

uniform lights_t lights_u;

in vec4 position_a;
in vec3 normal_a;

out vec3 normalFrag;
out vec4 positionFrag;
out vec4 shadowCoord[10];

void main() {
  normalFrag = normalize(matrices_u.nm * normal_a);
  positionFrag = position_a;
  gl_Position = matrices_u.mvp * position_a;
  if(lights_u.drawShadows == 1) {
    for(int i = 0; i < lights_u.count; i++) {
      shadowCoord[i] =
        lights_u.shadowMatrix[i] * position_a;
    }
  }
}
