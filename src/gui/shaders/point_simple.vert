#version 330
#extension GL_ARB_separate_shader_objects : enable

/* point_simple.vert */

struct matrices_t {
  mat4 mvp;
  mat4 mv;
};
uniform matrices_t matrices_u;

layout (location = 0) in vec4 position_a;
layout (location = 1) in vec4 color_a;

out vec4 color;

void main() {
  color = color_a;
  gl_PointSize = max(5.0 / abs((matrices_u.mv * position_a).z), 1.0);
  gl_Position = matrices_u.mvp * position_a;
}
