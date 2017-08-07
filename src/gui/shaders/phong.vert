#version 330

layout (std140) uniform matrices {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};

layout (std140) uniform material {
  vec4 Ka;
  vec4 Kd;
  vec4 Ks;
  int Ns;
} material;

layout (std140) uniform lights {
  int count;
  int directional[10];
  vec3 position[10];
  vec3 Ka[10];
  vec3 Kd[10];
  vec3 Ks[10];
} lights;

in vec4 position_a;
in vec3 normal_a;

out vec3 normalFrag;
out vec4 positionFrag;

void main(){
  normalFrag = normalize(nm * normal_a);
  positionFrag = position_a;
  gl_Position = mvp * position_a;
}
