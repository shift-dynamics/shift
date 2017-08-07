#version 330

uniform mat4 mvp;
uniform vec4 color_u;
uniform float scale;
in vec4 position_a;

out vec4 color;

void main() {
  color = color_u;
  gl_Position = mvp * scale * position_a;
}
