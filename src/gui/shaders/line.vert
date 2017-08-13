#version 330

uniform mat4 mvp_u;
uniform vec4 color_u;
uniform float scale_u = 1.f;
in vec4 position_a;

out vec4 color;

void main() {
  color = color_u;
  vec4 position = position_a;
  position.xyz *= scale_u;
  gl_Position = mvp_u * position;
}
