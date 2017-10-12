#version 420 core

layout (binding = 1) uniform sampler2D texture_displacement_u;

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
  float xScale;
  float yScale;
  float xRange;
  float yRange;
  vec2 viewport;
};

uniform terrain_t terrain_u;

in vec4 positionFrag;

in TES_OUT {
  vec2 tc;
  vec3 world_coord;
  vec3 eye_coord;
} fs_in;


void main(void) {

}
