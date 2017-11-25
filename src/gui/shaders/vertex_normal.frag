#version 330
/* vertex_normal.frag */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
  mat4 v;
  mat3 nm;
};
uniform matrices_t matrices_u;
uniform float scale_u;
uniform vec3 color_u;

out vec4 fragColor;

void main() {
  fragColor = vec4(color_u, 1.0);
}
