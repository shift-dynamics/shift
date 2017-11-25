#version 330
/* vertex_normal.vert */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};
uniform matrices_t matrices_u;
uniform float scale_u;
uniform vec3 color_u;

layout (location = 0) in vec4 position_a;
layout (location = 1) in vec3 normal_a;

out vec4 position;
out vec3 normal;

void main() {
  position = position_a;
  normal = normalize(normal_a);
}
