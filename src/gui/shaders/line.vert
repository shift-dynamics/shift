#version 330

/* line.vert */

uniform mat4 mvp_u;
uniform vec4 color_u;
uniform float scale_u;
layout (location = 0) in vec4 position_a;

out vec4 color;

void main() {
  color = color_u;
  gl_Position = mvp_u * vec4(scale_u * position.xyz, 1.f);
}
