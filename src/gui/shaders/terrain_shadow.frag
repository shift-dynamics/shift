#version 420 core

/* tertain_shadow.frag */

layout (binding = 1) uniform sampler2D texture_elevation_u;

out float fragmentDepth;

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

in vec4 positionFrag;

void main(void) {
  fragmentDepth = gl_FragCoord.z;
}
