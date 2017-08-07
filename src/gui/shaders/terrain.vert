#version 420 core

uniform sampler2D tex_displacement;

layout (std140) uniform matrices {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};

layout (std140) uniform material {
  uniform vec3 KaOverKd;
  uniform vec3 KsOverKd;
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

layout (std140) uniform terrain {
  float zMin;
  float zMax;
  vec2 viewport;
} terrain;

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
